import time,json,sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from mavros_msgs.msg import WaypointList,State
from mavros_msgs.srv import SetMode,CommandBool
from std_msgs.msg import Bool,Int32MultiArray,UInt64
from cdcl_umd_msgs.msg import TargetBoxArray,TargetBox
from rcl_interfaces.msg import Log
rclpy.init()
n=Node('sitl_mission_region_verifier')
t0=time.monotonic(); events=[]; seq=-1; armed=False; completed=False; sent=False
q=QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE)
latched=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def event(kind,value):
    row=dict(t=round(time.monotonic()-t0,3),seq=seq,kind=kind,value=value); events.append(row); print(json.dumps(row),flush=True)
def manifest(msg):
    global seq
    if seq != msg.current_seq:
        seq=msg.current_seq; event('cursor',seq)
def state(msg):
    global armed,completed
    if armed and not msg.armed: completed=True
    armed=msg.armed
n.create_subscription(WaypointList,'/uas11/mission/waypoints',manifest,latched)
n.create_subscription(State,'/uas11/state',state,q)
n.create_subscription(Bool,'/uas11/mosaic_capture_cmd',lambda m:event('mosaic',m.data),q)
n.create_subscription(Bool,'/uas11/fiducial_capture_cmd',lambda m:event('fiducial',m.data),q)
n.create_subscription(Bool,'/uas11/continuous_detection_state',lambda m:event('continuous',m.data),q)
n.create_subscription(Int32MultiArray,'/assessment/vantage_arrivals',lambda m:event('vantage',list(m.data)),q)
n.create_subscription(TargetBoxArray,'/uas11/assessment/roi_assignments',lambda m:event('assignment',dict(seq=m.seq,boxes=len(m.uav_target_boxes))),latched)
n.create_subscription(TargetBoxArray,'/uas11/target_detections/mosaic',lambda m:event('mosaic_image',len(m.source_img.data)),q)
n.create_subscription(TargetBoxArray,'/uas11/target_detections/vlm',lambda m:event('vlm',dict(boxes=len(m.uav_target_boxes),bytes=len(m.source_img.data))),q)
n.create_subscription(TargetBoxArray,'/uas11/target_detections',lambda m:event('detection',len(m.uav_target_boxes)),q)
n.create_subscription(UInt64,'/uas11/tracking/search_session',lambda m:event('search_session',m.data),latched)
n.create_subscription(UInt64,'/uas11/tracking/search_session_ack',lambda m:event('session_ack',m.data),q)
n.create_subscription(TargetBoxArray,'/uas11/tracking/target_locations',lambda m:event('tracking_input',len(m.uav_target_boxes)),q)
def log(m):
    if any(x in m.name for x in ['chimera_mission','survey','target_tracking','zoom']): event('log',m.msg)
n.create_subscription(Log,'/rosout',log,q)
pub=n.create_publisher(TargetBoxArray,'/uas11/static_targets',latched)
def call(kind,typ,req):
    c=n.create_client(typ,'/uas11/'+kind)
    if not c.wait_for_service(timeout_sec=15): raise RuntimeError(kind+' unavailable')
    f=c.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=15)
    if f.result() is None: raise RuntimeError(kind+' timed out')
    event('service',str(f.result())); return f.result()
for _ in range(30):rclpy.spin_once(n,timeout_sec=.1)
call('set_mode',SetMode,SetMode.Request(custom_mode='AUTO.MISSION'))
call('cmd/arming',CommandBool,CommandBool.Request(value=True))
while time.monotonic()-t0<360 and not completed:
    rclpy.spin_once(n,timeout_sec=.1)
    if seq==5 and not sent:
        box=TargetBox();box.data_source_id=990001;box.use_for_assessment=True
        fix=box.target_location_topo;fix.status.status=0;fix.latitude=39.2142211;fix.longitude=-76.63870;fix.altitude=13.8
        report=TargetBoxArray();report.system_id=11;report.seq=900001;report.uav_target_boxes=[box]
        pub.publish(report);event('seed_roi',990001);sent=True
checks={
 'mission_completed_and_disarmed':completed,
 'two_distinct_search_sessions':len(set(e['value'] for e in events if e['kind']=='search_session'))==2,
 'both_search_sessions_acknowledged':all(any(a['kind']=='session_ack' and a['value']==e['value'] for a in events) for e in events if e['kind']=='search_session'),
 'no_tracking_input_in_assess':not any(e['kind']=='tracking_input' and 7<=e['seq']<=12 for e in events),
 'tracking_input_in_both_search_sections':all(any(e['kind']=='tracking_input' and e['seq'] in ss for e in events) for ss in [[4,5],[14,15]]),
 'search_hold_observed':any(e['kind']=='cursor' and e['seq']==5 for e in events),
 'assessment_holds_observed':all(any(e['kind']=='cursor' and e['seq']==s for e in events)for s in [8,10]),
 'return_to_search_observed':any(e['kind']=='cursor' and e['seq']==15 for e in events),
 'mosaics_in_both_search_sections':all(any(e['kind']=='mosaic' and e['seq'] in ss for e in events)for ss in [[3,4,5],[13,14,15,16]]),
 'no_mosaics_in_assessment':not any(e['kind']=='mosaic' and 6<=e['seq']<=12 for e in events),
 'no_fiducial_in_active_sections':not any(e['kind']=='fiducial' and e['seq']>=3 for e in events),
 'vantage_announced_for_both_holds':all(any(e['kind']=='vantage' and e['value']==[11,s]for e in events)for s in [8,10]),
 'no_search_vantage_announcements':not any(e['kind']=='vantage' and e['value'][1]not in [-1,8,10]for e in events),
 'dispatcher_assignment_received':any(e['kind']=='assignment' and e['value']['seq'] in [8,10] and e['value']['boxes']>0 for e in events),
 'survey_dispatched':any(e['kind']=='log' and 'sent ROI survey' in e['value']for e in events),
 'continuous_frames_in_search_and_assess':all(any(e['kind']=='detection' and e['seq'] in ss for e in events)for ss in [[4,5],[7,8,9,10,11,12],[14,15]]),
 'search_mosaic_image_received':any(e['kind']=='mosaic_image' and e['value']>0 for e in events),
 'assessment_cancel_and_clear_observed':all(any(e['kind']=='log' and text in e['value']for e in events)for text in ['cancelling active ROI survey','ROI cleared']),
 'assessment_capture_received':any(e['kind']=='vlm' and e['seq'] in [8,10] and e['value']['bytes']>0 for e in events),
}
for hold,budget in [(8,25),(10,12)]:
    start=next((e['t']for e in events if e['kind']=='vantage' and e['value']==[11,hold]),None)
    stop=next((e['t']for e in events if e['seq']==hold and e['kind']=='log' and 'assessment window expired; discarded' in e['value']),None)
    checks[f'hold_{hold}_budget_{budget}s']=start is not None and stop is not None and abs(stop-start-budget)<1.0
print('RESULT '+json.dumps(dict(checks=checks,events=events)),flush=True)
n.destroy_node();rclpy.shutdown();sys.exit(0 if all(checks.values())else 1)
