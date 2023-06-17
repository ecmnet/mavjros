package glmapping;

public interface local2global extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "glmapping/local2global";
  static final java.lang.String _DEFINITION = "Header header\nint64  frame_id\nint32  pt_obs_count\nint32  pt_miss_count\ngeometry_msgs/Transform  T_w_l\ngeometry_msgs/Vector3[]  pts_obs_l\ngeometry_msgs/Vector3[]  pts_miss_l\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  long getFrameId();
  void setFrameId(long value);
  int getPtObsCount();
  void setPtObsCount(int value);
  int getPtMissCount();
  void setPtMissCount(int value);
  geometry_msgs.Transform getTWL();
  void setTWL(geometry_msgs.Transform value);
  java.util.List<geometry_msgs.Vector3> getPtsObsL();
  void setPtsObsL(java.util.List<geometry_msgs.Vector3> value);
  java.util.List<geometry_msgs.Vector3> getPtsMissL();
  void setPtsMissL(java.util.List<geometry_msgs.Vector3> value);
}
