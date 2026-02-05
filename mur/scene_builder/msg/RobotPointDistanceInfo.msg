# Info sulla distanza tra un punto di interesse sul robot e un collision object
# La distanza è calcolata come vettore dal punto al centro dell'oggetto (senza FCL)

string point_name                       # Nome del punto di interesse (es. "elbow", "wrist")
string link_name                        # Link del robot a cui è attaccato il punto
geometry_msgs/Point position            # Posizione del punto nel world frame
geometry_msgs/Vector3 linear_velocity   # Velocità lineare del punto nel world frame
string object_id                        # ID del collision object
geometry_msgs/Vector3 distance_vector   # Vettore dal punto di interesse al centro dell'oggetto
float64 distance                        # Distanza scalare (norma del distance_vector)
float64 object_characteristic_radius    # Raggio caratteristico dell'oggetto (per stimare distanza alla superficie)
geometry_msgs/Vector3 object_velocity   # Velocità lineare dell'oggetto nel world frame
