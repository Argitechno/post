import argparse
import asyncio
import rclpy
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args

def create_qos_profile(loss_mode: str, depth: int) -> QoSProfile:
    if loss_mode == 'lossless':
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=depth,
            history=HistoryPolicy.KEEP_ALL,
        )
    else:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=depth,
            history=HistoryPolicy.KEEP_LAST,
        )

async def spin_node_async(node):
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(0.001)  # tiny async sleep for tasks
    except SystemExit:
        rclpy.logging.get_logger("station_running").info('Node shutdown requested')
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main_template(station_class, default_name, add_args_fn=None):
    argv = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(description=f"Run {station_class.__name__} node")

    # Common args for all stations
    parser.add_argument('--loss_mode', choices=['lossy', 'lossless'], default='lossy', help='QoS loss mode')
    parser.add_argument('--depth', type=int, default=10, help='QoS queue depth')

    # Station-specific extra args
    if add_args_fn:
        add_args_fn(parser)
    
    args = parser.parse_args(argv[1:])
        
    rclpy.init()

    qos = create_qos_profile(args.loss_mode, args.depth)

    node = station_class(default_name, qos, args)

    asyncio.run(spin_node_async(node))
