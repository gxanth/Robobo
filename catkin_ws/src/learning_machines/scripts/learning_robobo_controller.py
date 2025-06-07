import argparse
from robobo_interface import SimulationRobobo, HardwareRobobo
from learning_machines import run_all_actions

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", action="store_true", help="Use the physical Robobo")
    parser.add_argument("--simulation", action="store_true", help="Use the CoppeliaSim simulator")
    parser.add_argument("--api-port", type=int, default=45100, help="ROS XML-RPC port")
    parser.add_argument("--tcpros-port", type=int, default=45101, help="ROS TCPROS port")
    args = parser.parse_args()

    if args.hardware:
        rob = HardwareRobobo(camera=True, xmlrpc_port=args.api_port, tcpros_port=args.tcpros_port)
    elif args.simulation:
        rob = SimulationRobobo()
    else:
        raise ValueError("You must pass either --hardware or --simulation")

    run_all_actions(rob)
