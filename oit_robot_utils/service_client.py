import rclpy
from rclpy.client import Client


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

    def call(self, request, timeout_sec: float = 5.0):
        """サービスを同期（ブロッキング）呼び出しし、結果を返す。"""
        logger = self.node.get_logger()

        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            logger.error(
                f"Service {self.service_name} is not available (timed out).")
            return None

        future = self.client.call_async(request)

        # 直列に spin してレスポンスを消化
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)

        if future.done():
            return future.result()

        logger.error(f"Service {self.service_name} call timed out.")
        return None

    def destroy(self):
        """クライアントリソースを破棄する。"""
        try:
            self.node.destroy_client(self.client)
        except Exception:
            pass
