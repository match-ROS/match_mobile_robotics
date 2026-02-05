# POI Debug Info message
# Contains detailed debug information for a single Point of Interest (POI)
# Used for debugging and visualization of repulsive velocity computation

# POI identification
string point_name                       # Nome POI (tcp, elbow, wrist, forearm_mid)
string link_name                        # Link del robot a cui appartiene il POI

# POI position and radius
geometry_msgs/Point position            # Posizione del POI nel world frame
float64 poi_radius                      # Raggio del POI (inflazione sfera robot)

# Obstacle information
string closest_obstacle_id              # ID dell'ostacolo più vicino
float64 obstacle_radius                 # Raggio caratteristico dell'ostacolo

# Distance information
geometry_msgs/Vector3 distance_vector   # Vettore dal POI al centro dell'ostacolo
float64 distance_raw                    # Distanza centro-centro (grezza)
float64 distance_effective              # Distanza superficie-superficie (effettiva)

# Repulsive velocity (BEFORE Jacobian projection)
geometry_msgs/Vector3 repulsive_velocity           # Velocità repulsiva cartesiana [m/s]
float64 repulsive_velocity_magnitude               # Modulo della velocità repulsiva [m/s]

# POI configuration
float64 weight                          # Peso del POI nella combinazione
bool is_active                          # true se il POI è nell'intervallo di influenza
