"""Idempotent lifecycle startup for map_server and AMCL, with bounded retries.
Used when the launcher disables automatic lifecycle startup. No /clock is needed.
"""
import time
import rclpy
from lifecycle_msgs.srv import GetState,ChangeState
from lifecycle_msgs.msg import Transition
rclpy.init();node=rclpy.create_node('localization_startup_check')

def call(client,req,timeout=5.):
    if not client.wait_for_service(timeout_sec=timeout):return None
    f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
    return f.result() if f.done() and f.exception() is None else None
try:
    for name in ['map_server','amcl']:
        get=node.create_client(GetState,f'/{name}/get_state')
        change=node.create_client(ChangeState,f'/{name}/change_state')
        for attempt in range(6):
            state=call(get,GetState.Request())
            if state is None:continue
            if state.current_state.id==3:break
            req=ChangeState.Request()
            if state.current_state.id==1:req.transition.id=Transition.TRANSITION_CONFIGURE
            elif state.current_state.id==2:req.transition.id=Transition.TRANSITION_ACTIVATE
            else:raise RuntimeError(f'{name}: unexpected lifecycle state {state.current_state.label}')
            call(change,req) # On lost response, re-read actual state before retrying.
        else:raise RuntimeError(f'{name}: failed to become active')
        print(name,'ACTIVE',flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
