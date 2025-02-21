from ultralytics import YOLO
import cv2
import numpy as np
from tqdm import tqdm


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Radio de la Tierra en kilómetros
    dLat = np.radians(lat2 - lat1)
    dLon = np.radians(lon2 - lon1)
    a = np.sin(dLat/2) * np.sin(dLat/2) + np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) * np.sin(dLon/2) * np.sin(dLon/2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R * c
    return distance 

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
    return vueloList
    
    
    
    

        
        
        
        
        
