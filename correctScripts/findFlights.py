from ultralytics import YOLO
import cv2
import numpy as np
from tqdm import tqdm
import json
import utm
import string
import numpy as np
import utm
import tkinter as tk
from tkinter import filedialog
import json
import os
import string
from tqdm import tqdm


def dms2dd(data):
    dd = float(data[0]) + float(data[1]) / 60 + float(data[2]) / (60 * 60)
    if data[3] == 'W' or data[3] == 'S':
        dd *= -1
    return dd

def get_image_pos_utm(data):
    # Obtiene las posiciones en el formato que sale con exiftools
    lat = data['GPSLatitude'].replace('\'', '').replace('"', '').split(' ')
    lng = data['GPSLongitude'].replace('\'', '').replace('"', '').split(' ')
    # Elimina la palabra 'deg' de los datos
    for v in lat:
        if v == 'deg':
            lat.pop(lat.index(v))
    for v in lng:
        if v == 'deg':
            lng.pop(lng.index(v))
    # Calcula la posición en coordenadas UTM
    pos = utm.from_latlon(dms2dd(lat), dms2dd(lng))

    return pos

def save_georef_matriz(data, desp_este=0, desp_norte=0, desp_yaw=0, offset_altura=0, modo_altura="relativo", dist=None, ans=None, sig=None):

    metadata = data
    if metadata['Model'] == "MAVIC2-ENTERPRISE-ADVANCED":
        img_height = int(data['ImageHeight'])
        img_width = int(data['ImageWidth'])
        tamano_pix = 0.000012
        dis_focal = 9 / 1000  # mavic 2 enterprice
        yaw = np.pi * (float(data["GimbalYawDegree"]) + float(desp_yaw)) / 180
        center = get_image_pos_utm(data)
        if modo_altura == "relativo":
            #altura = float(data['RelativeAltitude']) - float(offset_altura)
            if float(data['RelativeAltitude']) < 3:
                relAltitude = 3
            else:
                relAltitude = float(data['RelativeAltitude'])
            altura = relAltitude - float(offset_altura)
        else:
            altura = offset_altura
        GSD = tamano_pix * (altura) / dis_focal
        # Cálculo del desplazamiento debido al pitch de la cámara
        pitch = np.pi * (float(data["GimbalPitchDegree"])) / 180.0
        desp_pitch = altura * np.tan(-np.pi / 2 + pitch)
    elif metadata['Model'] == "M3T":
        img_height = int(data['ImageHeight'])
        img_width = int(data['ImageWidth'])
        tamano_pix = 0.000012
        dis_focal = 9 / 1000  # mavic 2 enterprice
        yaw = np.pi * (float(data["GimbalYawDegree"]) + float(desp_yaw)) / 180
        center = get_image_pos_utm(data)
        if modo_altura == "relativo":
            if float(data['RelativeAltitude']) < 3:
                relAltitude = 3
            else:
                relAltitude = float(data['RelativeAltitude'])
            altura = relAltitude - float(offset_altura)
        else:
            altura = offset_altura
        GSD = tamano_pix * (altura) / dis_focal
        # Cálculo del desplazamiento debido al pitch de la cámara
        pitch = np.pi * (float(data["GimbalPitchDegree"])) / 180.0
        desp_pitch = altura * np.tan(-np.pi / 2 + pitch)
    elif metadata['Model'] == "XT2":
        img_height = int(data['ImageHeight'])
        img_width = int(data['ImageWidth'])
        tamano_pix = 0.000012
        dis_focal = 9 / 1000  # mavic 2 enterprice
        yaw = np.pi * (float(data["GimbalYawDegree"]) + float(desp_yaw)) / 180
        center = get_image_pos_utm(data)
        if modo_altura == "relativo":
            altura = float(data['RelativeAltitude']) - float(offset_altura)
        else:
            altura = float(offset_altura)
        GSD = tamano_pix * (altura) / dis_focal
        # Cálculo del desplazamiento debido al pitch de la cámara
        pitch = np.pi * (float(data["GimbalPitchDegree"])) / 180.0
        desp_pitch = altura * np.tan(-np.pi / 2 + pitch)
    elif metadata['Model'] == "ZH20T":
        img_height = int(data['ImageHeight'])
        img_width = int(data['ImageWidth'])
        tamano_pix = 0.000012
        dis_focal = float(data['FocalLength'][:-2]) / 1000
        # yaw = np.pi * (float(data["FlightYawDegree"]) + desp_yaw) / 180
        yaw = np.pi * (float(data["GimbalYawDegree"]) + float(desp_yaw)) / 180
        pitch = np.pi * (float(data["GimbalPitchDegree"])) / 180.0

        try:
            distancia_laser = float(data["LRFTargetDistance"]) #if dist is not None else dist
            lat_laser = float(data["LRFTargetLat"])
            lon_laser = float(data["LRFTargetLon"])
            altura = distancia_laser * abs(np.sin(pitch))
            GSD = tamano_pix * altura / dis_focal
            if ans is not None and sig is not None:
                if float(sig["LRFTargetLat"]) < lat_laser < float(ans["LRFTargetLat"]):
                    lon_laser += float(sig["LRFTargetLon"]) + float(ans["LRFTargetLon"])
                    lon_laser /= 3
            usar_posicion_laser = False
            if usar_posicion_laser:
                center = utm.from_latlon(lat_laser, lon_laser)
                desp_pitch = 0
            else:
                center = get_image_pos_utm(data)
                desp_pitch = altura * np.tan(-np.pi / 2 + pitch)

        except:

            center = get_image_pos_utm(data)
            if modo_altura == "relativo":
                altura = float(data['RelativeAltitude']) - float(offset_altura)
            else:
                altura = float(offset_altura)
            GSD = tamano_pix * (altura) / dis_focal
            # Cálculo del desplazamiento debido al pitch de la cámara
            pitch = np.pi * (float(data["GimbalPitchDegree"])) / 180.0
            desp_pitch = altura * np.tan(-np.pi / 2 + pitch)
    else:
        print("===================================================")
        print("CÁMARA NO DEFINIDA")
        return

    mid_width = img_width / 2

    Matriz_y = np.zeros((img_height, img_width))
    Matriz_x = np.zeros((img_height, img_width))

    for pixel_y in range(img_height):
        distancia_y = (pixel_y - img_height / 2 + 0.5) * GSD
        Matriz_y[pixel_y, :] = np.ones(img_width) * -1 * distancia_y

    matriz_gsd_y = (np.append(Matriz_y[:, 0], Matriz_y[-1, 0]) - np.append(Matriz_y[0, 0], Matriz_y[:, 0]))
    matriz_gsd_x = matriz_gsd_y[1:-1]  # asumimos pixeles cuadrados
    matriz_gsd_x = np.append(matriz_gsd_x[0], matriz_gsd_x[:])

    for pixel_y in range(img_height):
        gsd_x = matriz_gsd_x[pixel_y]
        distancia_x = -gsd_x * (mid_width - 0.5)
        for pixel_x in range(img_width):
            Matriz_x[pixel_y, pixel_x] = distancia_x
            distancia_x = distancia_x + gsd_x

    # AJUSTAR OFFSET DEL GPS, VALORES REFERENCIALES
    Matriz_Este = Matriz_y * np.sin(yaw) - Matriz_x * np.cos(yaw) + center[0] + float(desp_este) + float(desp_pitch) * np.sin(yaw)
    Matriz_Norte = Matriz_y * np.cos(yaw) + Matriz_x * np.sin(yaw) + center[1] + float(desp_norte) + float(desp_pitch) * np.cos(yaw)

    #print(center[0], center[1])

    Matriz_zonas_1 = np.ones((img_height, img_width)) * center[2]
    Matriz_zonas_2 = np.ones((img_height, img_width)) * string.ascii_uppercase.find(center[3])

    matriz_puntos_utm = np.concatenate(
        [Matriz_Este[..., np.newaxis], Matriz_Norte[..., np.newaxis], Matriz_zonas_1[..., np.newaxis],
         Matriz_zonas_2[..., np.newaxis]], axis=-1)
    return matriz_puntos_utm

def saveGeoM(img_names, metadata_path, geonp_path, folder_path):
    for image_path in tqdm(img_names, desc="Generando Matrices Georeferenciadas de las imágenes"):
        # Carga la metadata de la imagen
        with open(f'{metadata_path}/{image_path[:-4]}.txt', 'r') as archivo:
            data = json.load(archivo)
            
        m = save_georef_matriz(data, data['offset_E_tot'], data['offset_N_tot'], data['offset_yaw'], data['offset_altura'])
        geo_name = f'{geonp_path}/{image_path[:-4]}.npy'
        np.save(geo_name, m)
    print(f"Matrices Georeferenciadas generadas para todas las imágenes de la carpeta {folder_path}")

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Radio de la Tierra en kilómetros
    dLat = np.radians(lat2 - lat1)
    dLon = np.radians(lon2 - lon1)
    a = np.sin(dLat/2) * np.sin(dLat/2) + np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.sin(dLon/2) * np.sin(dLon/2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R * c
    return distance 


def saveKMLFlights(path_imagenes, path_save, name):
    with open(path_save + f'/{name}.kml', 'w') as file:
            a = f'''<?xml version="1.0" encoding="UTF-8"?>
        <kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
        <Folder>
            <name>{path_save.split('/')[-1]}_{name}</name>
            '''
            file.write(a)
            vuelo_ant = ''
            for f_name in tqdm(path_imagenes, desc="Generando KML"):
                
                nombre = f_name[:-4]
                
                vuelo = 'cvat'
         
                # Carga la metadata de la imagen
                str_metada_file = f"{path_save}/metadata/{nombre}.txt"
                with open(str_metada_file) as metadata_file:
                    data2 = json.load(metadata_file)

                modo_altura = data2['modo_altura']
                m = save_georef_matriz(data2, data2['offset_E_tot'], data2['offset_N_tot'], data2['offset_yaw'], data2['offset_altura'], modo_altura)
                p1_ll = utm.to_latlon(m[0][0][0], m[0][0][1], int(m[0][0][2]), string.ascii_uppercase[int(m[0][0][3])])
                p2_ll = utm.to_latlon(m[0][-1][0], m[0][-1][1], int(m[0][-1][2]), string.ascii_uppercase[int(m[0][-1][3])])
                p3_ll = utm.to_latlon(m[-1][-1][0], m[-1][-1][1], int(m[-1][-1][2]),
                                    string.ascii_uppercase[int(m[-1][-1][3])])
                p4_ll = utm.to_latlon(m[-1][0][0], m[-1][0][1], int(m[-1][0][2]), string.ascii_uppercase[int(m[-1][0][3])])

                # Coordenadas para el kml
                cordinates = f"{str(p4_ll[1])},{str(p4_ll[0])},0 {str(p3_ll[1])},{str(p3_ll[0])},0 {str(p2_ll[1])},{str(p2_ll[0])},0 {str(p1_ll[1])},{str(p1_ll[0])},0 "

                txt_desplazamiento = "_DN" + str(data2['offset_N']) + \
                                    "_DE" + str(data2['offset_E']) + \
                                    "_DY" + str(data2['offset_yaw']) + \
                                    "_DV" + str(data2['desface_gps']) + \
                                    "_DA" + str(data2['offset_altura']) + \
                                    "_MA" + str(data2['modo_altura'])
                if vuelo != vuelo_ant:
                    if vuelo_ant != '':
                        a = f'''</Folder>'''
                        file.write(a)

                    a = f'''<Folder>
                            <name>{vuelo} - {txt_desplazamiento}</name>
                            '''
                    file.write(a)
                    vuelo_ant = vuelo

                txt_href = f'original_img/{nombre}.JPG'
                # print(nombre)
                a = f'''<GroundOverlay>
                <name>{nombre + txt_desplazamiento}</name>
                <Icon>
                    <href>{txt_href}</href>
                    <viewBoundScale>0.75</viewBoundScale>
                </Icon>
                <gx:LatLonQuad>
                    <coordinates>
                        {cordinates} 
                    </coordinates>
                </gx:LatLonQuad>
            </GroundOverlay>
            '''
                file.write(a)
            a = '''</Folder>
            </Folder>
        </kml>'''
            file.write(a)

    print(f"KML generado para la carpeta {path_save + '/' + path_save.split('/')[-1] + f'_{name}.kml'}")


def findFlights(path_root,folder_path, img_names, geonp_path, transformer):
    vueloList = []
    listCords = []
    idxVuelo = 0
    
    for image_path in tqdm(img_names, desc="Calculando lineas"):
        img = cv2.imread(folder_path + "/" + image_path)
        H, W, _ = img.shape

        # coordenada centro de la imagen
        xc = W // 2
        yc = H // 2
        
        geoImg = np.load(f"{geonp_path}/{image_path[:-4]}.npy")
        xc_utm, yc_utm = geoImg[yc][xc][0], geoImg[yc][xc][1]
        lonc, latc = transformer.transform(xc_utm, yc_utm)
        listCords.append((latc, lonc))
    
    latitudes, longitudes = zip(*listCords)

    # Calcular los cambios de dirección
    angles = []
    for i in range(1, len(listCords)):
        dy = latitudes[i] - latitudes[i-1]
        dx = longitudes[i] - longitudes[i-1]
        angle = np.arctan2(dy, dx) * (180 / np.pi)  # Convertir a grados
        angles.append(angle)

    # Identificar cambios significativos en la dirección
    angle_changes = [0]  # Primer punto no tiene cambio
    threshold = 10  # Umbral en grados para considerar cambio de dirección
    for i in range(1, len(angles)):
        delta_angle = abs(angles[i] - angles[i-1])
        if delta_angle > threshold:
            angle_changes.append(1)  # Cambio de dirección
        else:
            angle_changes.append(0)  # Dirección constante

    # Agrupar segmentos en base a los cambios de dirección
    segment_id = 0
    segments = []
    for i in range(len(angle_changes)):
        if i == 0 or angle_changes[i] == 1:
            segment_id += 1
        segments.append(segment_id)
        
        
            
            
            
    print(f"Vuelos: {segments}")
    print(f"Numero de vuelos: {len(segments)}")
    # for e, vuelo in enumerate(vueloList):
    #     saveKMLFlights(vuelo, path_root, f'Vuelo_{e}')
        
    for e, vuelo in enumerate(segments):
        print(f"Vuelo {e}: {vuelo} Imagenes")
        
        
