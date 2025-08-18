# PGV 센서로부터 수신한 모든 처리된 데이터를 포함
std_msgs/Header header

# PGV 센서 ID (코드상 0 또는 1)
int32 pgv_id

# 로봇의 상대 위치 및 자세 정보
float64 x_pos               # x 위치 (mm)
float64 y_pos               # y 위치 (mm)
float64 angle               # 각도 (degrees)

# 라인 추종 관련 정보
string direction            # 현재 선택된 주행 방향 ("No lane is selected" 등)
int32 color_lane_count      # 감지된 컬러 라인 수
int32 no_color_lane         # 컬러 라인 감지 여부 플래그
int32 no_pos                # 위치 감지 여부 플래그

# 태그(마커) 관련 정보
int32 tag_detected          # 태그 감지 여부 플래그
int32 tag_id                # 감지된 태그의 ID