# Jimmy Paputto 2026

uint8 band               # 0=L1, 1=L2/L5
uint8 jamming_state      # 0=Unknown, 1=OK, 2=Warning, 3=Critical
uint8 antenna_status     # 0=Init, 1=DontKnow, 2=Ok, 3=Short, 4=Open
uint8 antenna_power      # 0=Off, 1=On, 2=DontKnow
uint32 post_status
uint16 noise_per_ms
float32 agc_monitor
float32 cw_interference_suppression_level
int8  ofs_i
uint8 mag_i
int8  ofs_q
uint8 mag_q
