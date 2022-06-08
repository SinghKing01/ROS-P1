#!/usr/bin/env python

import rospy
import serial, time
import threading

global arduino

from sensor_msgs.msg import Range

# TO_DO: CREAR tres objetos Event, uno para cada thread de escritura por el puerto serie
object0 = threading.Event()
object1 = threading.Event()
object2 = threading.Event()

def requestMeasuresSensor(sensor,pub): # funcion de solicitud de dato del sensor y publicacin en ROS

  # creamos un mensaje tipo Range
  msg = Range()
  msg.header.frame_id = 'kobuki'
  msg.radiation_type = 1
  msg.field_of_view = 0
  msg.min_range = 0.2
  msg.max_range = 1.5

  while not rospy.is_shutdown(): # mientras no paremos el nodo

    arduino.write(sensor) # escribimos por el serie el nmero de sensor

    if sensor == '0':

      # TO_DO: BLOQUEAR el thread con el objeto Event correspondiente
      object0.wait();

      msg.range = range0 # rellenamos el mensaje con el rango recibido

    elif sensor == '1':

      # TO_DO: BLOQUEAR el thread con el objeto Event correspondiente
      object1.wait();

      msg.range = range1 # rellenamos el mensaje con el rango recibido

    elif sensor == '2':

      # TO_DO: BLOQUEAR el thread con el objeto Event correspondiente
      object2.wait();

      msg.range = range2 # rellenamos el mensaje con el rango recibido

    # rellenamos la cabecera del mensaje con la hora actual
    msg.header.stamp = rospy.get_rostime()

    #publicamos el mensaje usando el "publisher" que nos han pasado por parmetro
    pub.publish(msg)

  return

def readSensors(): # funcin de recepcin de los datos por el puerto serie

  global range0
  global range1
  global range2

  while not rospy.is_shutdown(): # mientras no paremos el nodo

    # Leemos los caracteres recibidos por el puerto
    buffer = arduino.read_until()

    # Si la longitud es igual a 8 (es el formato correcto de la info recibida)
    if len(buffer) == 8: # R X : X X X CR LF <-- estructura de string en el buffer (CR = carriage return; LF = line feed, new line)
                         # 0 1 2 3 4 5  6  7 <-- nmero de byte (char)

      #EXTRAER el valor del rango recibido y ALMACENARLO en rangeM en metros
      x0 = buffer[3]; # guardamos en x0 el primer caracter de la distancia
      x1 = buffer[4]; # guardamos en x1 el segundo caracter de la distancia
      x2 = buffer[5]; # guardamos en x2 el tercero caracter de la distancia

      # concatenos los caracteres de distancia y hacemos un casting a integer
      # dado que la distancia recibida esta en cm, hay que pasar a metros
      rangeM = int(x0+x1+x2)/100.0

      # Si el caracter recibido es correspondiente al del sensor 0
      if buffer[1] == '0':
        # Guardamos el valor recibido en la variable del sensor 0
        range0 = rangeM

        # TO_DO: LIBERAR el thread correspondiente al sensor 0
        object0.set();

      # Si el caracter recibido es correspondiente al del sensor 1
      elif buffer[1] == '1':
        # Guardamos el valor recibido en la variable del sensor 1
        range1 = rangeM

        # TO_DO: LIBERAR el thread correspondiente al sensor 1
        object1.set();

      # Si el caracter recibido es correspondiente al del sensor 2
      elif buffer[1] == '2':
        # Guardamos el valor recibido en la variable del sensor 2
        range2 = rangeM

        # TO_DO: LIBERAR el thread correspondiente al sensor 2
        object2.set();

  return

if __name__ == "__main__":

  # Inicializamos el nodo
  rospy.init_node('sharp_driver')

  # Leemos los parametros
  port = rospy.get_param('~port', '/dev/ttyUSB0')
  rospy.loginfo('Port: %s', port)

  baud = 9600

  # Creamos tres "publishers", uno para cada sensor, para publicar mensajes del tipo "Range"
  pub0 = rospy.Publisher('~range0', Range, queue_size=1)
  pub1 = rospy.Publisher('~range1', Range, queue_size=1)
  pub2 = rospy.Publisher('~range2', Range, queue_size=1)

  # Inicializamos la conexion serie
  rospy.loginfo("Connecting to the device ...")
  try:
    arduino = serial.Serial(port, baud)
    time.sleep(2)
  except serial.SerialException:
    rospy.logfatal("It was not possible to connect to the device")
    exit(0)
  rospy.loginfo("Successfully connected to the device!")

  # TO_DO: CREAR tres threads que ejecuten la funcion "requestMeasuresSensor", pasando 2 parmetros:
                                                                  # (1) char con el numero de sensor y
                                                                  # (2) el "publisher" correspondiente
  # Creamos el array threads para guardar todos los hilos creados
  threads = []

  # A cada thread a crear le asignamos la funcion requestMeasuresSensor con parametros indicados en args
  thread0 = threading.Thread(name='Thread0', target=requestMeasuresSensor, args=('0',pub0));
  thread1 = threading.Thread(name='Thread1', target=requestMeasuresSensor, args=('1',pub1));
  thread2 = threading.Thread(name='Thread2', target=requestMeasuresSensor, args=('2',pub2));

  # Guardamos los threads creados en el array threads
  threads.append(thread0);
  threads.append(thread1);
  threads.append(thread2);

  # TO_DO: INICIAR los tres threads creados
  thread0.start();
  thread1.start();
  thread2.start();

  # TO_DO: CREAR un thread que ejecute la funcin "readSensors" e INICIARLO
  thread3 = threading.Thread(name='Thread3', target=readSensors);
  threads.append(thread3); # Tambien lo guardamos en el array threads
  thread3.start(); # Lo lanzamos

  print('Threads running')

  # "spin" hasta que paremos el nodo.
  rospy.spin() # Los threads se estan ejecutando

  # TO_DO: ESPERAR a que acaben los cuatro threads
  for x in threads:
    x.join()

  # Cerramos la conexion serie
  arduino.close()

  print('All done')
