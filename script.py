import serial
import pymongo
from datetime import datetime
from pymongo.errors import PyMongoError

# Reemplaza con tus credenciales
username = "martin97"
password = "martin123456"
cluster_address = "cluster0.mongodb.net"
database_name = "arduino_db"
collection_name = "arduino_collection"

# Crear la cadena de conexión
connection_string = f"mongodb+srv://martin97:martin123456@cluster0.ymblkgt.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0"

mongo_client = pymongo.MongoClient(connection_string)
db = mongo_client[database_name]
collection = db[collection_name]

# Configura la conexión serial
serial_port = 'COM4'  # Reemplaza 'COM3' con tu puerto serial
baud_rate = 115200

# Bandera para detectar "PAYLOAD RECIBIDO OK"
payload_received = False

try:
    mongo_client = pymongo.MongoClient(connection_string)
    db = mongo_client[database_name]
    collection = db[collection_name]
    print("Conectado a MongoDB")
except PyMongoError as e:
    print(f"No se pudo conectar a MongoDB: {e}")
    exit()


try:
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Conectado al puerto serial: {serial_port}")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto serial: {e}")
    exit()

# Leer y guardar datos en bucle
try:
    while True:
        if ser.in_waiting > 0:
            sensor_value = ser.readline().decode('utf-8').strip()
            print(f"Valor del sensor: {sensor_value}")

            if payload_received:
                # Crear un documento para MongoDB
                data = {
                    'sensor_value': sensor_value,
                    'timestamp': datetime.utcnow()
                }

                # Insertar el documento en MongoDB
                collection.insert_one(data)
                print("Datos guardados en MongoDB")

                # Resetear la bandera
                payload_received = False
            elif sensor_value == "PAYLOAD RECIBIDO OK":
                # Activar la bandera
                payload_received = True
                print("Mensaje de control recibido, esperando el próximo valor para subir a MongoDB")
except PyMongoError as e:
    print(f"Error de MongoDB: {e}")
except Exception as e:
    print(f"Otro error ocurrió: {e}")
finally:
    # Cerrar la conexión serial al salir del bucle
    ser.close()
    print("Conexión serial cerrada")

    