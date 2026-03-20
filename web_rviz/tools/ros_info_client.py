import roslibpy


class RosInfoClient:
    def __init__(self, host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)

    def _call_service(self, name, service_type, request=None):
        service = roslibpy.Service(self.client, name, service_type)
        response = service.call(roslibpy.ServiceRequest(request or {}))
        return response or {}

    # ---------- connection ----------
    def connect(self):
        self.client.run()
        return self.client.is_connected

    def disconnect(self):
        self.client.terminate()

    # ---------- Topics ----------
    def get_topics(self):
        response = self._call_service('/rosapi/topics', 'rosapi_msgs/srv/Topics')
        return response.get('topics', []) if isinstance(response, dict) else []

    def get_topic_type(self, topic_name):
        response = self._call_service('/rosapi/topic_type', 'rosapi_msgs/srv/TopicType', {'topic': topic_name})
        return response.get('type', 'unknown') if isinstance(response, dict) else 'unknown'

    def get_topics_with_type(self):
        topics = self.get_topics()
        result = []

        for topic in topics:
            try:
                topic_type = self.get_topic_type(topic)
            except Exception:
                topic_type = 'unknown'

            result.append((topic, topic_type))

        return result

    # ---------- Nodes ----------
    def get_nodes(self):
        response = self._call_service('/rosapi/nodes', 'rosapi_msgs/srv/Nodes')
        return response.get('nodes', []) if isinstance(response, dict) else []

    # ---------- Params ----------
    def get_params(self):
        response = self._call_service('/rosapi/get_param_names', 'rosapi_msgs/srv/GetParamNames')
        return response.get('names', []) if isinstance(response, dict) else []

    def get_param_value(self, param_name):
        try:
            response = self._call_service('/rosapi/get_param', 'rosapi_msgs/srv/GetParam', {
                'name': param_name,
                'default_value': ''
            })
            value = response.get('value')
            print(f"Value of param '{param_name}': {value}")
            return value
        except Exception as e:
            print(f"Error getting param '{param_name}': {e}")
            return None

    def get_all_params_value(self, exclude=None, max_length=500):
        try:
            if exclude is None:
                exclude = [
                    'robot_description',
                    '/robot_description',
                    'robot_description_semantic',
                    '/robot_description_semantic',
                    'robot_description_kinematics',
                    '/robot_description_kinematics'
                ]

            params = self.get_params()
            param_dict = {}

            print('\nParameters:')
            print('----------------------------------')

            for param in params:
                if param in exclude:
                    continue

                try:
                    value = self.get_param_value(param)

                    if isinstance(value, str) and len(value) > max_length:
                        value_display = value[:max_length] + ' ... (truncated)'
                    else:
                        value_display = value

                    param_dict[param] = value
                    print(f'{param}: {value_display}')
                except Exception as e:
                    print(f'{param}: <error: {e}>')

            return param_dict
        except Exception as e:
            print(f'Error getting parameter list: {e}')
            return {}

    # ---------- Services ----------
    def get_services(self):
        response = self._call_service('/rosapi/services', 'rosapi_msgs/srv/Services')
        return response.get('services', []) if isinstance(response, dict) else []

    def get_service_type(self, service_name):
        response = self._call_service('/rosapi/service_type', 'rosapi_msgs/srv/ServiceType', {'service': service_name})
        return response.get('type', 'unknown') if isinstance(response, dict) else 'unknown'

    def get_services_with_type(self):
        services = self.get_services()
        result = []

        for service in services:
            try:
                service_type = self.get_service_type(service)
            except Exception:
                service_type = 'unknown'

            result.append((service, service_type))

        return result

    # ---------- Actions ----------
    def get_actions(self):
        response = self._call_service('/rosapi/action_servers', 'rosapi_msgs/srv/GetActionServers')
        if not isinstance(response, dict):
            return []
        return response.get('action_servers') or response.get('actions') or response.get('names') or []

    def get_action_type(self, action_name):
        try:
            response = self._call_service('/rosapi/action_type', 'rosapi_msgs/srv/ActionType', {'action': action_name})
        except Exception:
            response = self._call_service('/rosapi/action_type', 'rosapi_msgs/srv/ActionType', {'name': action_name})
        if not isinstance(response, dict):
            return 'unknown'
        return response.get('type') or response.get('action_type') or 'unknown'

    def get_actions_with_type(self):
        actions = self.get_actions()
        result = []

        for action in actions:
            try:
                action_type = self.get_action_type(action)
            except Exception:
                action_type = 'unknown'

            result.append((action, action_type))

        return result

    #------------- Message Info -------------
    def get_message_info(self, message_type):
        try:
            candidates = []
            for candidate in [message_type, message_type.replace('/msg/', '/')]:
                if candidate and candidate not in candidates:
                    candidates.append(candidate)

            info = None
            last_error = None
            for candidate in candidates:
                try:
                    info = self._call_service('/rosapi/message_details', 'rosapi_msgs/srv/MessageDetails', {'type': candidate})
                    if info:
                        break
                except Exception as exc:
                    last_error = exc

            if info is None and last_error is not None:
                raise last_error

            if not info or 'typedefs' not in info:
                print(f'No message info for {message_type}')
                return None

            typedefs = {t['type']: t for t in info['typedefs']}
            printed = set()

            def format_fields(msg_type):
                msg = typedefs[msg_type]

                fieldnames = msg['fieldnames']
                fieldtypes = msg['fieldtypes']
                fieldarraylen = msg['fieldarraylen']

                lines = []

                for name, ftype, arrlen in zip(fieldnames, fieldtypes, fieldarraylen):
                    if arrlen == -1:
                        field = f'{ftype} {name}'
                    elif arrlen == 0:
                        field = f'{ftype}[] {name}'
                    else:
                        field = f'{ftype}[{arrlen}] {name}'

                    lines.append(field)

                return lines

            def print_message(msg_type, is_root=False):
                if msg_type in printed:
                    return

                printed.add(msg_type)

                if is_root:
                    print(f'\n# {msg_type}')
                    print('----------------------------------')
                else:
                    print('\n' + '=' * 80)
                    print(f'MSG: {msg_type}')

                lines = format_fields(msg_type)

                for line in lines:
                    print(line)

                msg = typedefs[msg_type]
                for ftype in msg['fieldtypes']:
                    if ftype in typedefs and ftype not in printed:
                        print_message(ftype)

            print_message(message_type, True)
            return True

        except Exception as e:
            print(f'Error getting message info for {message_type}: {e}')
            return None

    # ---------- Print ----------
    def print_topics(self, type=False):
        if type:
            topics = self.get_topics_with_type()
            max_len = max(len(t[0]) for t in topics) if topics else 0

            print('\nTopics (with type):\n')
            for topic, topic_type in topics:
                print(f'{topic.ljust(max_len)}  ->  {topic_type}')
        else:
            print('\nTopics:\n')
            for topic in self.get_topics():
                print(topic)

    def print_services(self, type=False):
        if type:
            services = self.get_services_with_type()
            max_len = max(len(s[0]) for s in services) if services else 0

            print('\nServices (with type):\n')
            for service, service_type in services:
                print(f'{service.ljust(max_len)}  ->  {service_type}')
        else:
            print('\nServices:\n')
            for service in self.get_services():
                print(service)

    def print_nodes(self):
        print('\nNodes:\n')
        for node in self.get_nodes():
            print(node)

    def print_params(self):
        print('\nParams:\n')
        for param in self.get_params():
            print(param)

    def print_actions(self, type=False):
        if type:
            actions = self.get_actions_with_type()

            print('\nActions (with type):\n')

            if not actions:
                print('(none)')
                return

            max_len = max(len(a[0]) for a in actions)
            for name, action_type in actions:
                print(f'{name.ljust(max_len)}  ->  {action_type}')
        else:
            print('\nActions:\n')
            for action in self.get_actions():
                print(action)
