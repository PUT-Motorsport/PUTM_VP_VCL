# InvertersStatus.msg
# Statusy poszczególnych falowników: 0 = OK, 1 = SOFT_ERROR (Resetowanie), 2 = CRITICAL
uint8 fl_state
uint8 fr_state
uint8 rl_state
uint8 rr_state

bool is_limp_home_active