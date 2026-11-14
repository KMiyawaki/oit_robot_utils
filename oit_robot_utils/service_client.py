import time
from collections.abc import Callable
from typing import Any

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.client import Client
from std_srvs.srv import Trigger


class ServiceClientBlocking:
    """メインスレッド（手続き型ノード）専用の同期サービス呼び出しヘルパー。

    【⚠️ 厳禁事項 ⚠️】
    裏で Executor 等により常時 `spin()` されているノード（例:SensorListenerNode）では
    絶対に使用しないでください。

    【理由】
    ROS 2 では「単一ノード内での複数 spin 同時実行」が厳禁（未定義動作）です。
    スレッド間での rcl_wait 競合により、デッドロックします。

    ※ BasicNavigator のように直列（順番）に spin を呼ぶ構成は安全です。
    """

    def __init__(self, node, service_type, service_name: str):
        self.node = node
        self.service_name = service_name
        self.client: Client = self.node.create_client(
            service_type, service_name)

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def call(self, request, timeout_sec: float = 5.0, log_error=True):
        """サービスを同期（ブロッキング）呼び出しし、結果を返す。"""
        logger = self.node.get_logger()

        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            msg = f"Service {self.service_name} is not available (timed out)."
            if log_error:
                logger.error(msg)
            else:
                logger.warn(msg)
            return None

        future = self.client.call_async(request)

        # 直列に spin してレスポンスを消化
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)

        if future.done():
            return future.result()

        msg = f"Service {self.service_name} call timed out."
        if log_error:
            logger.error(msg)
        else:
            logger.warn(msg)
        return None

    def destroy(self):
        """クライアントリソースを破棄する。"""
        try:
            self.node.destroy_client(self.client)
        except Exception:
            pass


def wait_for_service_condition(
    node,
    srv_type: Any,
    srv_name: str,
    request: Any,
    predicate: Callable[[Any], bool],
    timeout_sec: float = 15.0,
    call_timeout_sec: float = 1.0,
    poll_interval: float = 5.0,
) -> bool:
    """指定サービスを呼び出し、predicate(response) が True を返すまで最大 timeout_sec 秒待機する。"""
    client = ServiceClientBlocking(node, srv_type, srv_name)
    deadline = time.monotonic() + timeout_sec
    try:
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                node.get_logger().error(
                    f'{srv_name} did not meet condition within {timeout_sec:.1f}s. Continuing...'
                )
                return False

            if not client.wait_for_service(timeout_sec=min(call_timeout_sec, remaining)):
                node.get_logger().info(f'Waiting for service {srv_name}...')
                continue

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                continue

            res = client.call(request, timeout_sec=min(
                call_timeout_sec, remaining), log_error=False)
            if res is not None and predicate(res):
                node.get_logger().info(f'{srv_name} is ready!')
                return True

            sleep_sec = min(poll_interval, max(
                0.0, deadline - time.monotonic()))
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
        return False
    finally:
        client.destroy()


def wait_for_lifecycle_node(
    node,
    node_name: str,
    target_state: int = State.PRIMARY_STATE_ACTIVE,
    timeout_sec: float = 10.0,
) -> bool:
    """単一ライフサイクルノードが target_state になるまで待機する。"""
    srv_name = f'/{node_name.strip("/")}/get_state'
    return wait_for_service_condition(
        node,
        GetState,
        srv_name,
        GetState.Request(),
        lambda res: res.current_state.id == target_state,
        timeout_sec=timeout_sec,
    )


def wait_for_lifecycle_manager(
    node,
    manager_name: str,
    timeout_sec: float = 15.0,
) -> bool:
    """lifecycle_manager の is_active が True になるまで待機する。"""
    srv_name = f'/{manager_name.strip("/")}/is_active'
    return wait_for_service_condition(
        node,
        Trigger,
        srv_name,
        Trigger.Request(),
        lambda res: bool(res.success),
        timeout_sec=timeout_sec,
    )
