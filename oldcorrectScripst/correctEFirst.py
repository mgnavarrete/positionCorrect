from ultralytics import YOLO
import os
import cv2
import numpy as np
from pyproj import CRS, Transformer
import pandas as pd
import json
from itertools import combinations
import math
import tkinter as tk
from tkinter import filedialog
from glob import glob


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Radio de la Tierra en kilómetros
    dLat = np.radians(lat2 - lat1)
    dLon = np.radians(lon2 - lon1)
    a = np.sin(dLat/2) * np.sin(dLat/2) + np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.sin(dLon/2) * np.sin(dLon/2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R * c
    return distance


def calcular_centroide(puntos):
    suma_x = sum(p[0] for p in puntos)
    suma_y = sum(p[1] for p in puntos)
    count = len(puntos)
    return (int(round(suma_x / count)), int(round(suma_y / count)))


def calcular_area_poligono(puntos):
    n = len(puntos)
    area = 0
    for i in range(n):
        j = (i + 1) % n
        area += puntos[i][0] * puntos[j][1]
        area -= puntos[j][0] * puntos[i][1]
    area = abs(area) / 2.0
    return area

def centroide(puntos):
    x = sum(punto[0] for punto in puntos) / len(puntos)
    y = sum(punto[1] for punto in puntos) / len(puntos)
    return x, y

def angulo_con_respecto_al_centro(punto, centro):
    return math.atan2(punto[1] - centro[1], punto[0] - centro[0])

def ordenar_puntos(puntos):
    centro = centroide(puntos)
    puntos = sorted(puntos, key=lambda punto: angulo_con_respecto_al_centro(punto, centro))

    return [puntos[0], puntos[2], puntos[1], puntos[3]]

def ordenar_puntos(puntos):
    # Ordenar los puntos basándose en su coordenada x
    puntos = sorted(puntos, key=lambda punto: punto[0])

    # Separar los puntos en dos grupos basados en su posición x
    izquierda = puntos[:2]
    derecha = puntos[2:]

    # Dentro de cada grupo, ordenarlos por su coordenada y
    izquierda = sorted(izquierda, key=lambda punto: punto[1])
    derecha = sorted(derecha, key=lambda punto: punto[1], reverse=True)

    # El orden final es: superior izquierdo, inferior izquierdo, inferior derecho, superior derecho
    return [izquierda[0], izquierda[1], derecha[0], derecha[1]]

# Función para dividir la columna 'poly' en dos columnas 'lat' y 'lon'
def split_poly_into_lat_lon(df, poly_column):
    df[['lon', 'lat']] = df[poly_column].str.split(',', expand=True).astype(float)
    return df

# Tu función findClosest modificada
def findClosest(x1, y1, df, poly):
    df = split_poly_into_lat_lon(df,poly)
    x_utm, y_utm = geoImg[y1][x1][0], geoImg[y1][x1][1]
    lon, lat = transformer.transform(x_utm, y_utm)

    # Calcula la distancia usando una función vectorizada
    df['distance'] = df.apply(lambda row: haversine_distance(lat, lon, row['lat'], row['lon']), axis=1)

    # Encuentra el punto más cercano
    closest_row = df.loc[df['distance'].idxmin()]
    closest_name = closest_row['name']
    min_distance = closest_row['distance']

    return closest_name, min_distance, poly

def closest_values_sorted(lst, n=3):
    if len(lst) < n:
        return lst  # Retorna la lista completa si es más corta que n

    lst.sort()  # Ordena la lista

    min_diff = float('inf')
    closest_subset = []

    # Itera a través de la lista, considerando secuencias de n valores consecutivos
    for i in range(len(lst) - n + 1):
        current_subset = lst[i:i + n]
        diff = max(current_subset) - min(current_subset)

        if diff < min_diff:
            min_diff = diff
            closest_subset = current_subset

    return closest_subset

def anguloNorte(lat1, lon1, lat2, lon2):
    # Convierte latitud y longitud de grados a radianes
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    # Calcula el cambio en las coordenadas
    dlon = lon2 - lon1

    # Calcula el angulo
    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(dlon))
    initial_bearing = np.arctan2(x, y)

    # Convierte el angulo de radianes a grados
    initial_bearing = np.degrees(initial_bearing)

    # Normaliza el angulo
    bearing = (initial_bearing + 360) % 360

    return bearing


# Función para seleccionar múltiples directorios
def select_directories():
    path_root = filedialog.askdirectory(title='Seleccione el directorio raíz')
    while path_root:
        list_folders.append(path_root)
        path_root = filedialog.askdirectory(title='Seleccione otro directorio o cancele para continuar')
    if not list_folders:
        raise Exception("No se seleccionó ningún directorio")


list_folders = []
list_images = ["FIT_C41_V13_DJI_20231216020920_0256_T_20231216020920.JPG","FIT_C41_V13_DJI_20231216020920_0255_T_20231216020920.JPG","FIT_C41_V13_DJI_20231216020357_0062_T_20231216020357.JPG","FIT_C41_V10_DJI_20231216014044_0154_T_20231216014044.JPG","FIT_C41_V10_DJI_20231216014045_0155_T_20231216014045.JPG", "FIT_C41_V08_DJI_20231216013559_0021_T_20231216013559.JPG"]
model_path = 'best.pt'

# Iniciar Tkinter
root = tk.Tk()
root.withdraw()

print("Seleccione la tabla KML...")
csv_file_path = filedialog.askopenfile(title='Seleccione Tabla KML')
if not csv_file_path:
        raise Exception("No se seleccionó ningúna Tabla KML")
print("Tabla KML seleccionada")

# Llamar a la función para seleccionar directorios
print("Seleccione el directorio raíz...")
select_directories()
print("Directorio raíz seleccionado")

for path_root in list_folders:
    print(f"Procesando Carpeta:{path_root}")

    # Construir rutas a los subdirectorios
    folder_path = os.path.join(path_root, 'original_img')  # Para las imágenes originales
    imgsFolder = os.path.join(path_root, 'cvat')
    geonp_path = os.path.join(path_root, 'georef_numpy')  # Para archivos numpy georeferenciados
    metadata_path = os.path.join(path_root, 'metadata')  # Para archivos JSON de metadatos
    metadatanew_path = os.path.join(path_root, 'metadata')  # Para archivos JSON con offset_yaw modificado

    img_names = os.listdir(imgsFolder)
    img_names.sort()


    zone_number = 19
    zone_letter = 'S'

    # Define la proyección UTM (incluyendo la zona y el hemisferio)
    utm_crs = CRS(f"+proj=utm +zone={zone_number} +{'+south' if zone_letter > 'N' else ''} +ellps=WGS84")

    # Define la proyección de latitud/longitud
    latlon_crs = CRS("EPSG:4326")

    # Crear un objeto Transformer para la transformación de coordenadas
    transformer = Transformer.from_crs(utm_crs, latlon_crs, always_xy=True)

    if not os.path.exists(metadatanew_path):
            os.mkdir(metadatanew_path)
    # Preprocesar coordenadas en el DataFrame
    print("Cargando datos de KML...")

    df = pd.read_csv(csv_file_path)
    print("Datos cargados")

    print("Cargando modelo YOLO..")
    model = YOLO(model_path)
    print("Modelo cargado")

    print("Iniciando análisis de imágenes...")
    # Crear un diccionario para mapear nombres a coordenadas de polyname



    coordenadas_dict = df.set_index('name').to_dict(orient='index')
    for image_path in img_names:

        keypoint = []

        img = cv2.imread(folder_path + "/" + image_path)

        H, W, _ = img.shape
        img_resized = cv2.resize(img, (640, 640))
        results = model(img_resized)
        oeList = []
        alturaList = []
        for result in results:
            if result.masks is not None:
                for j, mask in enumerate(result.masks.data):
                    mask = mask.cpu().numpy() * 255
                    mask = cv2.resize(mask, (W, H))
                    img = cv2.resize(img, (W, H))
                    # Convertir la máscara a una imagen binaria
                    _, thresholded = cv2.threshold(mask, 25, 255, cv2.THRESH_BINARY)

                    # Encontrar contornos
                    contours, _ = cv2.findContours(thresholded.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if image_path in list_images:    
                        cv2.imwrite(f'masks/{image_path[:-4]}_{j}.png', mask)
                    if contours:
                        # Encuentra el contorno más grande
                        largest_contour = max(contours, key=cv2.contourArea)

                        # Aproximación del polígono
                        epsilon = 0.05* cv2.arcLength(largest_contour, True)
                        approx_polygon = cv2.approxPolyDP(largest_contour, epsilon, True)
                        approx_polygon = sorted(approx_polygon, key=lambda x: x[0][0])
                        approx_polygon = np.array(approx_polygon, dtype=int)

                        # print(f"approx_polygon: {approx_polygon}")
                        if len(approx_polygon) > 3:
                            # print(f"Procesando Imagen: {image_path}"
                            x, y, w, h = cv2.boundingRect(largest_contour)
                            approx_polygon = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=int)
                    
                            x1 = approx_polygon[0][0]
                            y1 = approx_polygon[0][1]
                            x2 = approx_polygon[1][0]
                            y2 = approx_polygon[1][1]
                            x3 = approx_polygon[2][0]
                            y3 = approx_polygon[2][1]
                            x4 = approx_polygon[3][0]
                            y4 = approx_polygon[3][1]

                            puntos = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
                            puntos_ordenados = ordenar_puntos(puntos)
                            x1, y1 = puntos_ordenados[0]
                            x2, y2 = puntos_ordenados[1]
                            x3, y3 = puntos_ordenados[2]
                            x4, y4 = puntos_ordenados[3]
                            
                            # Calcular distancia entre puntos 1 y 2 y 3 y 4 en pixeles
                            distancia1 = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                            distancia2 = math.sqrt((x4 - x3)**2 + (y4 - y3)**2)
                            
                            print(f"distancia1: {distancia1}")
                            print(f"distancia2: {distancia2}")
                            # Calcular el promedio de las distancias
                            avg_distancia = (distancia1 + distancia2) / 2
                            print(f"avg_distancia: {avg_distancia}")
                   

                            
                            area = calcular_area_poligono(puntos_ordenados)
                            if image_path in list_images:
                                print(f"area: {area}")
                            if area > 1000 or avg_distancia > 66:
                                # Convertir a formato numpy
                                puntos_np = np.array([(x1,y1),(x2,y2),(x3,y3),(x4,y4)], np.int32)
                                puntos_np = puntos_np.reshape((-1, 1, 2))

                                xc = int(round((x1 + x2 + x3 + x4) / 4))
                                yc = int(round((y1 + y2 + y3 + y4) / 4))
                                cv2.circle(img, (xc, yc), 5, (255, 255, 255), -1)
                                cv2.circle(img, (x1, y1), 5, (0, 0, 255), -1)
                                cv2.circle(img, (x4, y4), 5, (255, 0, 255), -1)
                                cv2.circle(img, (x2, y2), 5, (255, 0, 0), -1)
                                cv2.circle(img, (x3, y3), 5, (255, 255, 0), -1)
                                cv2.polylines(img, [puntos_np], isClosed=True, color=(0, 255, 0), thickness=3)
                            
                                geoImg = np.load(f"{geonp_path}/{image_path[:-4]}.npy")

                                x_utm, y_utm = geoImg[yc][xc][0], geoImg[yc][xc][1]
                                lonImg, latImg = transformer.transform(x_utm, y_utm)

                                oeList.append([xc, yc, lonImg, latImg])
                                # cv2.imwrite(f"results/{image_path[:-4]}_E.png", img)
                                


        if len(oeList) > 0:
            # Calcular el promedio de las lonitudes
            promedio_lon = sum([c[2] for c in oeList]) / len(oeList)

            # Dividir los centroides en dos grupos
            grupo_arriba = [c for c in oeList if c[2] < promedio_lon]
            grupo_abajo = [c for c in oeList if c[2] >= promedio_lon]
            print(f"Grupo arriba: {grupo_arriba}")
            print(f"Grupo abajo: {grupo_abajo}")
            
            if len(grupo_arriba) > 0 and len(grupo_abajo) > 0:
                print("Ambos grupos tienen elementos")
                for i in grupo_abajo:
                    x ,y,_,_ = i
                    cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
                for i in grupo_arriba:
                    x ,y,_,_ = i
                    cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

                # Calcular el centroide para cada grupo
                centroide_grupo_arriba = calcular_centroide([(c[0], c[1]) for c in grupo_arriba])
                centroide_grupo_abajo = calcular_centroide([(c[0], c[1]) for c in grupo_abajo])

                #dibujar linea entre centroides
                cv2.line(img, centroide_grupo_arriba, centroide_grupo_abajo, (255, 255, 255), 2)

                # Dibujar los centroides en la imagen
                cv2.circle(img, centroide_grupo_arriba, 5, (255, 255, 0), -1)
                cv2.circle(img, centroide_grupo_abajo, 5, (255, 0, 255), -1)

                xu, yu = centroide_grupo_arriba
                xd, yd = centroide_grupo_abajo
                xu_utm, yu_utm = geoImg[yu][xu][0], geoImg[yu][xu][1]
                xd_utm, yd_utm = geoImg[yd][xd][0], geoImg[yd][xd][1]
                lonu, latu = transformer.transform(xu_utm, yu_utm)
                lond, latd = transformer.transform(xd_utm, yd_utm)


                # Calcular Distancia entre centroides
                distancia = haversine_distance(latu, lonu, latd, lond)


                namep1, minp1, polynamep1 = findClosest(xu,yu,df,'point')
                namep2, minp2, polynamep2 = findClosest(xd,yd,df, 'point')

                # Obtener lon y lat directamente del diccionario
                lonKMLu, latKMLu= coordenadas_dict[namep1][polynamep1].split(",")
                lonKMLd, latKMLd= coordenadas_dict[namep2][polynamep2].split(",")

                lonKMLu, latKMLu = float(lonKMLu), float(latKMLu)
                lonKMLd, latKMLd = float(lonKMLd), float(latKMLd)


                # Calcular la diferencia de longitud y la distancia este-oeste
                diff_lonu = lonKMLu - lonu
                diff_lond = lonKMLd - lond
                # Earth's circumference along the equator in kilometers
                earth_circumference_km = 40075.0

                # Convert offset from degrees to kilometers (1 degree = Earth's circumference / 360)
                offset_kmu = diff_lonu * (earth_circumference_km / 360)
                offset_kmd = diff_lond * (earth_circumference_km / 360)

                # Convert kilometers to meters
                offset_polyu = offset_kmu * 1000
                offset_polyd = offset_kmd * 1000

                offset_oe = (offset_polyu + offset_polyd)/2

            # Condición cuando solo el grupo de arriba tiene elementos
            elif len(grupo_arriba) > 0 and len(grupo_abajo) <= 0:
                print("Solo el grupo de arriba tiene elementos")

                for i in grupo_arriba:
                    x ,y,_,_ = i
                    cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

                # Calcular el centroide para cada grupo
                centroide_grupo_arriba = calcular_centroide([(c[0], c[1]) for c in grupo_arriba])

                xu, yu = centroide_grupo_arriba

                xu_utm, yu_utm = geoImg[yu][xu][0], geoImg[yu][xu][1]

                lonu, latu = transformer.transform(xu_utm, yu_utm)


                namep1, minp1, polynamep1 = findClosest(xu,yu,df,'point')


                # Obtener lon y lat directamente del diccionario
                lonKMLu, latKMLu= coordenadas_dict[namep1][polynamep1].split(",")


                lonKMLu, latKMLu = float(lonKMLu), float(latKMLu)



                # Calcular la diferencia de longitud y la distancia este-oeste
                diff_lonu = lonKMLu - lonu

                # Earth's circumference along the equator in kilometers
                earth_circumference_km = 40075.0

                # Convert offset from degrees to kilometers (1 degree = Earth's circumference / 360)
                offset_kmu = diff_lonu * (earth_circumference_km / 360)

                # Convert kilometers to meters
                offset_polyu = offset_kmu * 1000

                offset_oe = offset_polyu

            # Condición cuando solo el grupo de abajo tiene elementos
            elif len(grupo_arriba) <= 0 and len(grupo_abajo) > 0:
                print("Solo el grupo de abajo tiene elementos")

                for i in grupo_abajo:
                    x ,y,_,_ = i
                    cv2.circle(img, (x, y), 5, (0, 0, 255), -1)


                # Calcular el centroide para cada grupo
                centroide_grupo_abajo = calcular_centroide([(c[0], c[1]) for c in grupo_abajo])



                xd, yd = centroide_grupo_abajo

                xd_utm, yd_utm = geoImg[yd][xd][0], geoImg[yd][xd][1]

                lond, latd = transformer.transform(xd_utm, yd_utm)

                namep2, minp2, polynamep2 = findClosest(xd,yd,df, 'point')

                # Obtener lon y lat directamente del diccionario
                lonKMLd, latKMLd= coordenadas_dict[namep2][polynamep2].split(",")

                lonKMLd, latKMLd = float(lonKMLd), float(latKMLd)


                # Calcular la diferencia de longitud y la distancia este-oeste
                diff_lond = lonKMLd - lond
                # Earth's circumference along the equator in kilometers
                earth_circumference_km = 40075.0

                # Convert offset from degrees to kilometers (1 degree = Earth's circumference / 360)
                offset_kmd = diff_lond * (earth_circumference_km / 360)

                # Convert kilometers to meters
                offset_polyd = offset_kmd * 1000

                offset_oe = offset_polyd


            else:
                offset_oe = 0
        else:
            offset_oe = 0

        if image_path in list_images:
            cv2.imwrite(f"results/{image_path[:-4]}_E.png", img)
        
        print(f"El offset_E de {image_path}: {offset_oe}")
        # Abre el archivo JSON en modo lectura
        with open(f'{metadata_path}/{image_path[:-4]}.txt', 'r') as archivo:
            data = json.load(archivo)

        # Modifica el valor de "offset_oe" con el número deseado
        data['offset_E'] = offset_oe
        # print(f"El offset_yaw de {image_path}: {offset_yaw}")
        # Abre el archivo JSON en modo escritura
        with open(f'{metadatanew_path}/{image_path[:-4]}.txt', 'w') as archivo:
            # Escribe el diccionario modificado de nuevo en el archivo JSON
            json.dump(data, archivo, indent=4)

    print(f"Carpeta {path_root} OK")

print("Todas la carpetas OK")