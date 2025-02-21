from functions import *

def findFlights(folder_path, img_names, geonp_path, transformer):
    vueloList = []
    lastCords = [None, None]
    idxVuelo = 0
    latOrder = None
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
            if latOrder == None:
                latdiff = latc - lastCords[0]
                londiff = lonc - lastCords[1]
                print(f"latdiff: {latdiff}")
                print(f"londiff: {londiff}")
                
                if latdiff < londiff:
                    latOrder = True
                    umb = latdiff * 0.05
                    
                else:
                    latOrder = False
                    umb = londiff * 0.05
                    
            elif latOrder:
                latdiff = latc - lastCords[0]
                if -umb < latdiff < umb:
                    vueloList[idxVuelo].append([image_path])
                    
                else:
                    idxVuelo += 1
                    vueloList.append([image_path])
            else:
                latdiff = latc - lastCords[0]
                if -umb < latdiff < umb:
                    vueloList[idxVuelo].append([image_path])
                else:
                    idxVuelo += 1
                    vueloList.append([image_path])
                    
            lastCords[0] = latc
            lastCords[1] = lonc
                    
         
            
            
        else:
            lastCords[0] = latc
            lastCords[1] = lonc
            vueloList.append([image_path])
            
    print(f"Numero de vuelos: {len(vueloList)}")
    print(f"Vuelos: {vueloList}")
    return vueloList
    
    
    
    

        
        
        
        
        
