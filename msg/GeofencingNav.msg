# Jimmy Paputto 2026
# Geofencing navigation status

uint32 itow                  # GPS time of week (ms)
uint8  geofencing_status     # 0=NotAvailable, 1=Active
uint8  number_of_geofences
uint8  combined_state        # 0=Unknown, 1=Inside, 2=Outside
uint8[4] geofences_status    # Per-fence status (0=Unknown, 1=Inside, 2=Outside)
