import json
import subprocess
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Deque, Dict, Optional, Set, Type

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from webrviz_interfaces.srv import GetInterfaceText


TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}


@dataclass
class GoalRecord:
    goal_id_msg: Any
    status_code: int = GoalStatus.STATUS_UNKNOWN
    state: str = "unknown"
    result_requested: bool = False


@dataclass
class ObservedAction:
    kind: str
    action_name: str
    event_topic: str
    action_type: Type[Any]
    publisher: Any
    result_client: Any
    goals: Dict[str, GoalRecord] = field(default_factory=dict)
    completed_ids: Deque[str] = field(default_factory=lambda: deque(maxlen=128))
    completed_lookup: Set[str] = field(default_factory=set)

    def mark_completed(self, goal_id: str) -> None:
        if goal_id in self.completed_lookup:
            return
        if len(self.completed_ids) == self.completed_ids.maxlen:
            expired = self.completed_ids.popleft()
            self.completed_lookup.discard(expired)
        self.completed_ids.append(goal_id)
        self.completed_lookup.add(goal_id)
        self.goals.pop(goal_id, None)


class MoveItObserverNode(Node):
    def __init__(self) -> None:
        super().__init__("webrviz_moveit_observer")

        self.declare_parameter("plan_action_name", "/move_action")
        self.declare_parameter("execute_action_name", "/execute_trajectory")
        self.declare_parameter("plan_event_topic", "/webrviz/moveit/plan_event")
        self.declare_parameter("execute_event_topic", "/webrviz/moveit/execute_event")
        self.declare_parameter("interface_text_service_name", "/webrviz/get_interface_text")

        plan_action_name = self.get_parameter("plan_action_name").get_parameter_value().string_value or "/move_action"
        execute_action_name = self.get_parameter("execute_action_name").get_parameter_value().string_value or "/execute_trajectory"
        plan_event_topic = self.get_parameter("plan_event_topic").get_parameter_value().string_value or "/webrviz/moveit/plan_event"
        execute_event_topic = self.get_parameter("execute_event_topic").get_parameter_value().string_value or "/webrviz/moveit/execute_event"
        interface_text_service_name = self.get_parameter("interface_text_service_name").get_parameter_value().string_value or "/webrviz/get_interface_text"

        self._observed: Dict[str, ObservedAction] = {
            "plan": self._create_observed_action(
                kind="plan",
                action_name=plan_action_name,
                event_topic=plan_event_topic,
                action_type=MoveGroup,
            ),
            "execute": self._create_observed_action(
                kind="execute",
                action_name=execute_action_name,
                event_topic=execute_event_topic,
                action_type=ExecuteTrajectory,
            ),
        }

        self.create_service(GetInterfaceText, interface_text_service_name, self._handle_get_interface_text)

        self.get_logger().info(
            "observing MoveIt actions: "
            f"plan={plan_action_name} -> {plan_event_topic}, "
            f"execute={execute_action_name} -> {execute_event_topic}, "
            f"interface_text_service={interface_text_service_name}"
        )

    def _create_observed_action(self, kind: str, action_name: str, event_topic: str, action_type: Type[Any]) -> ObservedAction:
        publisher = self.create_publisher(String, event_topic, 10)
        result_client = self.create_client(action_type.Impl.GetResultService, f"{action_name}/_action/get_result")
        observed = ObservedAction(
            kind=kind,
            action_name=action_name,
            event_topic=event_topic,
            action_type=action_type,
            publisher=publisher,
            result_client=result_client,
        )

        self.create_subscription(
            GoalStatusArray,
            f"{action_name}/_action/status",
            self._make_status_callback(observed),
            10,
        )
        self.create_subscription(
            action_type.Impl.FeedbackMessage,
            f"{action_name}/_action/feedback",
            self._make_feedback_callback(observed),
            10,
        )
        return observed

    def _make_status_callback(self, observed: ObservedAction) -> Callable[[GoalStatusArray], None]:
        def callback(message: GoalStatusArray) -> None:
            for status in message.status_list:
                goal_id = self._goal_id_to_string(status.goal_info.goal_id)
                if goal_id in observed.completed_lookup:
                    continue

                next_state = self._status_to_state(status.status)
                record = observed.goals.get(goal_id)
                previous_status = record.status_code if record else None
                if record is None:
                    record = GoalRecord(goal_id_msg=status.goal_info.goal_id)
                    observed.goals[goal_id] = record
                record.goal_id_msg = status.goal_info.goal_id
                record.status_code = status.status
                record.state = next_state

                if previous_status != status.status and status.status not in TERMINAL_STATUSES:
                    self._publish_event(
                        observed,
                        goal_id=goal_id,
                        status_code=status.status,
                        state=next_state,
                        source="status",
                    )

                if status.status in TERMINAL_STATUSES and not record.result_requested:
                    self._request_result(observed, goal_id)
        return callback

    def _make_feedback_callback(self, observed: ObservedAction) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            goal_id = self._goal_id_to_string(message.goal_id)
            if goal_id in observed.completed_lookup:
                return

            record = observed.goals.get(goal_id)
            if record is None:
                record = GoalRecord(goal_id_msg=message.goal_id)
                observed.goals[goal_id] = record

            if record.state == "executing":
                return

            record.goal_id_msg = message.goal_id
            record.status_code = GoalStatus.STATUS_EXECUTING
            record.state = "executing"
            self._publish_event(
                observed,
                goal_id=goal_id,
                status_code=GoalStatus.STATUS_EXECUTING,
                state="executing",
                source="status",
            )
        return callback

    def _request_result(self, observed: ObservedAction, goal_id: str) -> None:
        record = observed.goals.get(goal_id)
        if record is None:
            return

        if not observed.result_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"result service unavailable for {observed.action_name}: {goal_id}")
            return

        request = observed.action_type.Impl.GetResultService.Request()
        request.goal_id = record.goal_id_msg
        record.result_requested = True
        future = observed.result_client.call_async(request)
        future.add_done_callback(lambda done: self._handle_result(observed, goal_id, done))

    def _handle_result(self, observed: ObservedAction, goal_id: str, future: Future) -> None:
        record = observed.goals.get(goal_id)
        try:
            response = future.result()
        except Exception as exc:
            if record is not None:
                record.result_requested = False
            self.get_logger().warning(f"result request failed for {observed.action_name}: {goal_id}: {exc}")
            return

        status_code = int(getattr(response, "status", GoalStatus.STATUS_UNKNOWN))
        result = getattr(response, "result", None)
        error_code = self._extract_error_code(result)
        planning_time = self._extract_planning_time(result)
        self._publish_event(
            observed,
            goal_id=goal_id,
            status_code=status_code,
            state=self._status_to_state(status_code),
            source="result",
            error_code=error_code,
            planning_time=planning_time,
        )
        observed.mark_completed(goal_id)

    def _publish_event(
        self,
        observed: ObservedAction,
        *,
        goal_id: str,
        status_code: int,
        state: str,
        source: str,
        error_code: Optional[int] = None,
        planning_time: Optional[float] = None,
    ) -> None:
        payload = {
            "kind": observed.kind,
            "action_name": observed.action_name,
            "goal_id": goal_id,
            "status_code": int(status_code),
            "status_text": self._status_to_state(status_code),
            "state": state,
            "error_code": error_code,
            "planning_time": planning_time,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "source": source,
        }

        message = String()
        message.data = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
        observed.publisher.publish(message)
        self.get_logger().info(
            f"published {observed.kind} event | source={source} | state={state} | goal_id={goal_id}"
        )

    def _handle_get_interface_text(self, request: GetInterfaceText.Request, response: GetInterfaceText.Response) -> GetInterfaceText.Response:
        candidates = self._type_name_candidates(request.type_name)
        last_error = ""

        for candidate in candidates:
            try:
                completed = subprocess.run(
                    ["ros2", "interface", "show", candidate],
                    capture_output=True,
                    text=True,
                    timeout=8,
                    check=False,
                )
            except Exception as exc:  # pragma: no cover
                last_error = str(exc)
                continue

            output = (completed.stdout or "").strip()
            if completed.returncode == 0 and output:
                response.success = True
                response.message = ""
                response.raw_text = output
                self.get_logger().info(f"provided interface text for {candidate}")
                return response

            stderr = (completed.stderr or "").strip()
            if stderr:
                last_error = stderr

        response.success = False
        response.message = last_error or f"unable to resolve interface text for {request.type_name}"
        response.raw_text = ""
        self.get_logger().warning(response.message)
        return response

    @staticmethod
    def _type_name_candidates(type_name: str) -> list[str]:
        trimmed = (type_name or "").strip()
        if not trimmed:
            return []

        candidates = [trimmed]
        if "/msg/" in trimmed:
            candidates.append(trimmed.replace("/msg/", "/"))
        elif "/srv/" in trimmed:
            candidates.append(trimmed.replace("/srv/", "/"))
        elif "/action/" in trimmed:
            candidates.append(trimmed.replace("/action/", "/"))

        unique: list[str] = []
        for candidate in candidates:
            if candidate and candidate not in unique:
                unique.append(candidate)
        return unique

    @staticmethod
    def _goal_id_to_string(goal_id_msg: Any) -> str:
        values = list(getattr(goal_id_msg, "uuid", []))
        return "".join(f"{int(value):02x}" for value in values)

    @staticmethod
    def _status_to_state(status_code: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: "unknown",
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        return mapping.get(int(status_code), f"unknown({status_code})")

    @staticmethod
    def _extract_error_code(result: Any) -> Optional[int]:
        error_code = getattr(result, "error_code", None)
        value = getattr(error_code, "val", None)
        return int(value) if isinstance(value, int) else None

    @staticmethod
    def _extract_planning_time(result: Any) -> Optional[float]:
        value = getattr(result, "planning_time", None)
        if isinstance(value, (int, float)):
            return float(value)
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
