from ultralytics import YOLO
import cv2
import numpy as np
from tqdm import tqdm
import json
import utm
import string
from saveGeoMatriz import *


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Radio de la Tierra en kilómetros
    dLat = np.radians(lat2 - lat1)
    dLon = np.radians(lon2 - lon1)
    a = np.sin(dLat/2) * np.sin(dLat/2) + np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.sin(dLon/2) * np.sin(dLon/2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R * c
    return distance 


def saveKMLFlights(path_imagenes, path_save, name):
    with open(path_save + '/' + path_save.split('/')[-1] + f'_{name}.kml', 'w') as file:
            a = f'''<?xml version="1.0" encoding="UTF-8"?>
        <kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
        <Folder>
            <name>{path_save.split('/')[-1]}_PA</name>
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

    print(f"KML generado para la carpeta {path_save + '/' + path_save.split('/')[-1] + '.kml'}")


def findFlights(folder_path, img_names, geonp_path, transformer):
    vueloList = []
    lastCords = [None, None]
    idxVuelo = 0
    FirstLine = True
    umb = None
    for image_path in tqdm(img_names, desc="Calculando lineas"):
        img = cv2.imread(folder_path + "/" + image_path)
        H, W, _ = img.shape

        # coordenada centro de la imagen
        xc = W // 2
        yc = H // 2
        
        geoImg = np.load(f"{geonp_path}/{image_path[:-4]}.npy")
        xc_utm, yc_utm = geoImg[yc][xc][0], geoImg[yc][xc][1]
        lonc, latc = transformer.transform(xc_utm, yc_utm)
        
        if lastCords[0] != None:            
            if FirstLine:
                pendiente = (latc - lastCords[0]) / (lonc - lastCords[1])
                FirstLine = False
                umb = pendiente * 0.01
            else: 
                pendiente = (latc - lastCords[0]) / (lonc - lastCords[1])
                if -umb < pendiente < umb:
                    vueloList[idxVuelo].append(image_path)
                else:
                    idxVuelo += 1
                    vueloList.append([image_path])
            
         
            
            
        else:
            lastCords[0] = latc
            lastCords[1] = lonc
            vueloList.append([image_path])
            
    
    print(f"Vuelos: {vueloList}")
    print(f"Numero de vuelos: {len(vueloList)}")
    print(f"Generando KML de los vuelos")
    for e, vuelo in enumerate(vueloList):
        saveKMLFlights(vuelo, folder_path, f'V{e}')
    return vueloList
    
    
    
    

        
        
        
        
        
