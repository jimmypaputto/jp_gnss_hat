# Jimmy Paputto 2026
# Single satellite information

uint8 gnss_id        # 0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 4=IMES, 5=QZSS, 6=GLONASS
uint8 sv_id          # Satellite vehicle ID
uint8 cno            # Carrier-to-noise ratio (dBHz)
int8  elevation      # degrees
int16 azimuth        # degrees
uint8 quality        # 0=NoSignal..7=CodeAndCarrierLocked3
bool  used_in_fix
bool  healthy
bool  diff_corr      # Differential correction available
bool  eph_avail      # Ephemeris available
bool  alm_avail      # Almanac available
