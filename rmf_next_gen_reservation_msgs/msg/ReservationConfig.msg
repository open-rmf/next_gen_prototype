# ReservationConfig.msg

# Reservation configuration loaded by the destination server.

# Size of an individual occupancy grid cell, in meters.
float32 grid_size

# Predefined safe sets robots are allowed to be sent to.
NamedRegion[] safe_sets

# Backup parking spots a robot can be diverted to while it waits.
NamedRegion[] parking_spots
