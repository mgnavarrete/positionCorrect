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
            altura = float(data['RelativeAltitude']) - float(offset_altura)
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
            altura = float(data['RelativeAltitude']) - float(offset_altura)
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

# Función para seleccionar múltiples directorios
def select_directories():
    
    path_root = filedialog.askdirectory(title='Seleccione el directorio raíz')
    while path_root:
        list_folders.append(path_root)
        path_root = filedialog.askdirectory(title='Seleccione otro directorio o cancele para continuar')
    if not list_folders:
        raise Exception("No se seleccionó ningún directorio")
    
def saveGeoM(img_names, metadata_path, geonp_path, folder_path):
    for image_path in tqdm(img_names, desc="Generando Matrices Georeferenciadas de las imágenes"):
        # Carga la metadata de la imagen
        with open(f'{metadata_path}/{image_path[:-4]}.txt', 'r') as archivo:
            data = json.load(archivo)
            
        m = save_georef_matriz(data, data['offset_E_tot'], data['offset_N_tot'], data['offset_yaw'], data['offset_altura'])
        geo_name = f'{geonp_path}/{image_path[:-4]}.npy'
        np.save(geo_name, m)
    print(f"Matrices Georeferenciadas generadas para todas las imágenes de la carpeta {folder_path}")

if __name__ == "__main__":
    list_folders = []
    list_images = []

    # Iniciar Tkinter
    root = tk.Tk()
    root.withdraw()

    # Llamar a la función para seleccionar directorios
    print("Seleccione el directorio raíz...")
    select_directories()
    print("Directorio raíz seleccionado")

    for path_root in list_folders:
        # Agregar a path root PP
        path_root = path_root + "PP"
        print(f"Procesando Carpeta:{path_root}")
        # Construir rutas a los subdirectorios
        folder_path = os.path.join(path_root, 'original_img')  # Para las imágenes originales
        imgsFolder = os.path.join(path_root, 'cvat')
        geonp_path = os.path.join(path_root, 'georef_numpy')  # Para archivos numpy georeferenciados
        metadata_path = os.path.join(path_root, 'metadata')  # Para archivos JSON de metadatos
        metadatanew_path = os.path.join(path_root, 'metadata')  # Para archivos JSON con offset_yaw modificado

        img_names = os.listdir(imgsFolder)
        img_names.sort()
    
        saveGeoM(img_names, metadata_path, geonp_path, path_root)   