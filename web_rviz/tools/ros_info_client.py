import roslibpy
import threading

class RosInfoClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)

    # ---------- connection ----------
    def connect(self):
        self.client.run()
        return self.client.is_connected

    def disconnect(self):
        self.client.terminate()

    # ---------- Topics ----------
    def get_topics(self):
        topics = self.client.get_topics()

        if isinstance(topics, dict):
            return topics.get("topics", [])
        return topics

    def get_topics_with_type(self):
        topics = self.get_topics()
        result = []

        for topic in topics:
            try:
                t = self.client.get_topic_type(topic)
            except Exception:
                t = "unknown"

            result.append((topic, t))

        return result

    # ---------- Nodes ----------
    def get_nodes(self):
        nodes = self.client.get_nodes()

        if isinstance(nodes, dict):
            return nodes.get("nodes", [])
        return nodes

    # ---------- Params ----------
    def get_params(self):
        params = self.client.get_params()

        if isinstance(params, dict):
            return params.get("names", [])
        return params

    # ---------- Services ----------
    def get_services(self):
        services = self.client.get_services()

        if isinstance(services, dict):
            return services.get("services", [])
        return services

    def get_services_with_type(self):
        services = self.get_services()
        result = []

        for service in services:
            try:
                t = self.client.get_service_type(service)
            except Exception:
                t = "unknown"

            result.append((service, t))

        return result
    
    # ------Actions (auto detect) -------
    def get_actions(self):

        topics = self.get_topics()

        suffix = {
            "/goal",
            "/result",
            "/feedback",
            "/status",
            "/cancel"
        }

        actions = set()

        for topic in topics:
            for s in suffix:
                if topic.endswith(s):
                    action = topic[:-len(s)]
                    actions.add(action)

        return sorted(actions)

    def get_actions_with_type(self):

        topics = self.get_topics()
        result = []

        for topic in topics:

            if topic.endswith("/goal"):

                action_name = topic[:-5]

                try:
                    goal_type = self.client.get_topic_type(topic)

                    # Example:
                    # control_msgs/FollowJointTrajectoryActionGoal
                    action_type = goal_type.replace("ActionGoal", "")

                except Exception:
                    action_type = "unknown"

                result.append((action_name, action_type))

        return result
    
    #------------- Message Info -------------
    def get_message_info(self, message_type):
        try:
            info = self.client.get_message_details(message_type)

            if not info or "typedefs" not in info:
                print(f"No message info for {message_type}")
                return None

            typedefs = {t["type"]: t for t in info["typedefs"]}
            printed = set()

            def format_fields(msg_type):
                msg = typedefs[msg_type]

                fieldnames = msg["fieldnames"]
                fieldtypes = msg["fieldtypes"]
                fieldarraylen = msg["fieldarraylen"]

                lines = []

                for name, ftype, arrlen in zip(fieldnames, fieldtypes, fieldarraylen):

                    # 处理数组
                    if arrlen == -1:
                        field = f"{ftype} {name}"
                    elif arrlen == 0:
                        field = f"{ftype}[] {name}"
                    else:
                        field = f"{ftype}[{arrlen}] {name}"

                    lines.append(field)

                return lines

            def print_message(msg_type, is_root=False):
                if msg_type in printed:
                    return

                printed.add(msg_type)

                if is_root:
                    print(f"\n# {msg_type}")
                    print("----------------------------------")
                else:
                    print("\n" + "=" * 80)
                    print(f"MSG: {msg_type}")

                lines = format_fields(msg_type)

                for l in lines:
                    print(l)

                # 递归展开子message
                msg = typedefs[msg_type]
                for ftype in msg["fieldtypes"]:
                    if ftype in typedefs and ftype not in printed:
                        print_message(ftype)

            print_message(message_type, True)

            return True

        except Exception as e:
            print(f"Error getting message info for {message_type}: {e}")
            return None
    
    # ------------ Params Value -------------    
    def get_param_value(self, param_name):
        try:
            value = self.client.get_param(param_name)
            print(f"Value of param '{param_name}': {value}")
            return value
        except Exception as e:
            print(f"Error getting param '{param_name}': {e}")
            return None
        
    def get_all_params_value(self, exclude=None, max_length=500):
        try:
            if exclude is None:
                exclude = [
                    "/robot_description",
                    "/robot_description_semantic",
                    "/robot_description_kinematics"
                ]

            params = self.client.get_params()
            param_dict = {}

            print("\nParameters:")
            print("----------------------------------")

            for param in params:

                if param in exclude:
                    continue

                try:
                    value = self.client.get_param(param)

                    # 处理过长字符串
                    if isinstance(value, str) and len(value) > max_length:
                        value_display = value[:max_length] + " ... (truncated)"
                    else:
                        value_display = value

                    param_dict[param] = value

                    print(f"{param}: {value_display}")

                except Exception as e:
                    print(f"{param}: <error: {e}>")

            return param_dict

        except Exception as e:
            print(f"Error getting parameter list: {e}")
            return {}

    # ---------- Print ----------
    def print_topics(self, type=False):

        if type:
            topics = self.get_topics_with_type()
            max_len = max(len(t[0]) for t in topics)

            print("\nTopics (with type):\n")
            for topic, t in topics:
                print(f"{topic.ljust(max_len)}  ->  {t}")

        else:
            print("\nTopics:\n")
            for topic in self.get_topics():
                print(topic)

    def print_services(self, type=False):

        if type:
            services = self.get_services_with_type()
            max_len = max(len(s[0]) for s in services)

            print("\nServices (with type):\n")
            for service, t in services:
                print(f"{service.ljust(max_len)}  ->  {t}")

        else:
            print("\nServices:\n")
            for service in self.get_services():
                print(service)

    def print_nodes(self):
        print("\nNodes:\n")
        for node in self.get_nodes():
            print(node)

    def print_params(self):
        print("\nParams:\n")
        for param in self.get_params():
            print(param)
            
    def print_actions(self, type=False):

        if type:

            actions = self.get_actions_with_type()

            print("\nActions (with type):\n")

            if not actions:
                print("(none)")
                return

            max_len = max(len(a[0]) for a in actions)

            for name, t in actions:
                print(f"{name.ljust(max_len)}  ->  {t}")

        else:

            print("\nActions:\n")

            for action in self.get_actions():
                print(action)