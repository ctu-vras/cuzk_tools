from collections import defaultdict, OrderedDict

import fiona
import numpy as np
from tqdm import tqdm
import shutil
import matplotlib.pyplot as plt
import os

""" 
    Czechia SJTSK bounds are:
        XMin: -927817.5799999982
        YMin: -1236550.607999999
        XMax: -422329.256000001
        YMax: -933179.3900000006

    Split the one file containing the complete information of Czechia into rectangles similarly
    as with the DMR5G data. In fact use the exact same rectangles.

    If an object lies between multiple rectangles, assign it to all of them (least evil solution).

    V X to jde po 2500 (  -740 000 ->   -742 500).
    V Y to jde po 2000 (-1 040 000 -> -1 042 000).
"""

USE_ORIGINAL_CATEGORIES = False

RAM_LIMIT = 1800000

H = 8000
W = 10000

XMIN_CZ = -927817.58
YMIN_CZ = -1236550.608
XMAX_CZ = -422329.256
YMAX_CZ = -933179.39

XMIN = int(np.floor(XMIN_CZ) - np.floor(XMIN_CZ)%W)
YMIN = int(np.floor(YMIN_CZ) - np.floor(YMIN_CZ)%H)
XMAX = int(np.ceil(XMAX_CZ) + (W - np.ceil(XMAX_CZ)%W))
YMAX = int(np.ceil(YMAX_CZ) + (H - np.ceil(YMAX_CZ)%H))

buildings = ['VodojemVezovy','VezVezovitaNastavba','VetrnyMotor','VetrnyMlyn',
             'TovarniKomin','TezniVez','StozarLanoveDrahy','StozarElektrickehoVedeni',
             'Zamek','VezovitaStavba','Tribuna','StavebniObjektZakryty','Silo',
             'RozvodnaTransformovna','RozvalinaZricenina','NadzemniZasobniNadrz',
             'KulnaSklenikFoliovnikPristresek','ChladiciVez','Hrad','BudovaJednotlivaNeboBlokBudov']

roads = ['Ulice','SilniceVeVystavbe','SilniceNeevidovana','SilniceDalnice','ParkovisteOdpocivka']

rails = ['ZeleznicniVlecka','ZeleznicniTrat','TramvajovaDraha','ZeleznicniTocnaPresuvna','Kolejiste']

footways = ['Ulice','Tunel','Pesina','Lavka','Cesta','Lavka_b']

water = ['VodniTok','Brod','VodniPlocha','UsazovaciNadrz','PozemniNadrz',]

forest = ['VyznamnyNeboOsamelyStromLesik','LiniovaVegetace','LesniPudaSeStromyKategorizovana',
          'LesniPudaSKrovinatymPorostem','LesniPudaSKosodrevinou','LesniPudaSeStromy',]

antiforest = ['LesniPrusek']

agriculture = ['Vinice','OvocnySadZahrada','OrnaPudaAOstatniDaleNespecifikovanePlochy','Chmelnice',]

untraversable = ['UlozneMisto','Skladka','SkalniUtvary','SesuvPudySut','Raseliniste',
                 'PovrchovaTezbaLom','Letiste','Hrbitov','Heliport','BazinaMocal']

traversable = ['TrvalyTravniPorost','OkrasnaZahradaPark',]

obstacles = ['Zabrana','OsamelyBalvanSkalaSkalniSuk','MohylaPomnikNahrobek','MeteorologickaStaniceDefinicniBod',
            'KrizSloupKulturnihoVyznamu','Zed','StupenSraz','SkupinaBalvanu','RokleVymol','PataTerennihoUtvaru',
            'LyzarskyMustek','HradbaValBastaOpevneni','DopravnikovyPas','DalkovyProduktovodDalkovePotrubi',
            'Raseliniste_b','BudovaJednotlivaNeboBlokBudov_b']

typulice_dict = dict(roads=['026','926'], footways=['025','125','225','925'])

categories = [buildings,roads,rails,footways,water,forest,antiforest,agriculture,untraversable,traversable,obstacles]
categories_str = ['buildings','roads','rails','footways','water','forest',
                  'antiforest','agriculture','untraversable','traversable','obstacles']

original_categories = [[c] for c in sum(categories, [])]
original_categories_str = sum(categories, [])

geom_types = ['Point', 'MultiLineString', 'MultiPolygon'] 

""" def get_file_schema(gpkg_file):
    with fiona.open(gpkg_file, 'r', driver='GPKG') as gpkg:
        schema = gpkg.schema
    
    return schema """

def get_layer_schemas(gpkg_file):
    layer_names = fiona.listlayers(gpkg_file)
    layer_schemas = {}
    
    for layer_name in layer_names:
        with fiona.open(gpkg_file, layer=layer_name) as layer:
            layer_schemas[layer_name] = (layer.schema, layer.crs)

    return layer_schemas

def get_simple_schema(geom_type):
    return dict(properties=OrderedDict(fid_zbg="str"), geometry=geom_type)

def get_simple_crs():
    return dict(init='epsg:5514')

def get_layer_name(cat,geom):
    return cat + '_' + geom.lower()

def create_gpkg_file(fn, layer_schemas):
    for layer_name, (schema, crs) in layer_schemas.items():
        with fiona.open(fn, 'w', driver='GPKG', layer=layer_name, schema=schema, crs=crs) as gpkg:
            pass

def create_categorized_gpkg_file(fn, categories, geom_types):
    for category in categories:
        for geom_type in geom_types:
            layer_name = get_layer_name(category, geom_type)
            with fiona.open(fn, 'w', driver='GPKG', layer=layer_name, schema=get_simple_schema(geom_type), crs=get_simple_crs()) as gpkg:
                pass

def get_rectangle_ranges(h=8000,w=10000):

    x_n = int(np.ceil((XMAX-XMIN)/w))
    y_n = int(np.ceil((YMAX-YMIN)/h))

    rect_ranges = [(0., 0., 0., 0.)] * x_n * y_n    # xmin, ymin, xmax, ymax

    for i in range(x_n):
        for j in range(y_n):
            ind = j + y_n * i
            rect_ranges[ind] = [int(XMIN + i*w), int(YMIN + j*h), int(XMIN + (1+i)*w), int(YMIN + (1+j)*h)]
    
    return rect_ranges

def get_rects(coords_x_min, coords_y_min, coords_x_max, coords_y_max ,h,w):

    rects = []

    x_n = max(int(np.ceil((coords_x_max-coords_x_min)/w)),1)
    y_n = max(int(np.ceil((coords_y_max-coords_y_min)/h)),1)

    """ coords_x_min = np.min(coords[:,0])
    coords_y_min = np.min(coords[:,1])
    coords_x_max = np.max(coords[:,0])
    coords_y_max = np.max(coords[:,1]) """

    x_low = int(coords_x_min - (coords_x_min % w))
    y_low = int(coords_y_min - (coords_y_min % h))
    x_hi  = int(coords_x_max + (w - (coords_x_max % w)))
    y_hi  = int(coords_y_max + (h - (coords_y_max % h)))

    x_n = int((x_hi - x_low)/w)
    y_n = int((y_hi - y_low)/h)

    for i in range(x_n):
        for j in range(y_n):
            rects.append((x_low + w*i,
                          y_low + h*j,
                          x_low + w*(i+1),
                          y_low + h*(j+1)))

    return rects

def visualize_layer(gpkg_file, layer_name):
    data = np.empty((100000000,2),float)
    c = 0
    with fiona.open(gpkg_file, layer=layer_name) as layer:
        for feature in tqdm(layer):
            #if feature['properties']['typulice_k'] not in ['026','025','225','926']:
            #    print(feature['properties']['typulice_k'])
            geom_type = feature['geometry']['type']
            coords = feature['geometry']['coordinates']

            if geom_type == "MultiPolygon":
                if len(coords[0]) > 1:
                    for i in range(len(coords[0])):
                        coords_split = np.array(coords[0][i])
                        data[c:c+coords_split.shape[0],:] = coords_split
                        c += coords_split.shape[0]
                else:
                    coords = np.array(coords)[0,0]
                    data[c:c+coords.shape[0],:] = coords
                    c += coords.shape[0]

            elif geom_type == "MultiLineString":
                coords = np.array(coords)[0]
                data[c:c+coords.shape[0],:] = coords
                c += coords.shape[0]
            
    data = data[:c,:]
    print(data.shape)
    plt.scatter(data[:,0],data[:,1])
    plt.show()

def split_gpkg_into_files(gpkg_file, fn_preposition, use_original_categories):
    #layer_schemas = get_layer_schemas(gpkg_file)

    global categories, categories_str, original_categories, original_categories_str, geom_types
    rect_ranges = get_rectangle_ranges(H,W)

    print("Creating GPKG files.")

    first_fn = os.environ['HOME'] + '/.ros/cache/cuzk_tools/topography/gpkg_files/'+ fn_preposition + '_' \
                                                + str(rect_ranges[0][0])[1:] + '_' \
                                                + str(rect_ranges[0][1])[1:] + '_' \
                                                + str(rect_ranges[0][2])[1:] + '_' \
                                                + str(rect_ranges[0][3])[1:]
    
    if use_original_categories:
        create_categorized_gpkg_file(first_fn, original_categories_str, geom_types)
    else:
        create_categorized_gpkg_file(first_fn, categories_str, geom_types)

    for rect_range in tqdm(rect_ranges[1:]):
        fn = os.environ['HOME'] + '/.ros/cache/cuzk_tools/topography/gpkg_files/'  + fn_preposition + '_' \
                                                + str(rect_range[0])[1:] + '_' \
                                                + str(rect_range[1])[1:] + '_' \
                                                + str(rect_range[2])[1:] + '_' \
                                                + str(rect_range[3])[1:]
        shutil.copy2(first_fn, fn)

    print("Parsing layers.")
    categories_to_use =  original_categories if use_original_categories else categories
    categories_str_to_use =  original_categories_str if use_original_categories else categories_str

    for i,category in tqdm(enumerate(categories_to_use)):

        data = defaultdict(lambda: defaultdict(list))
        category_str = categories_str_to_use[i]

        c = 0

        for layer_name in tqdm(category):

            try:
                with fiona.open(gpkg_file, layer=layer_name) as layer:

                    print("\nLayer    : {}".format(layer_name))
                    print("Num items: {:,}".format(len(layer)))

                    for feature in tqdm(layer):

                        if layer_name == 'Ulice':
                            if category_str == 'roads':
                                if not feature['properties']['typulice_k'] in typulice_dict[category_str]:
                                    continue

                            elif category_str == 'footways':
                                if not feature['properties']['typulice_k'] in typulice_dict[category_str]:
                                    continue

                            else:
                                raise NotImplementedError("This should not happen, probably some new type of ulice has been used. Edit 'typulice_dict'.")

                        coords = feature['geometry']['coordinates']
                        geom_type = feature['geometry']['type']

                        if geom_type == 'Point':
                            coords_x_min = coords[0]
                            coords_y_min = coords[1]
                            coords_x_max = coords[0]
                            coords_y_max = coords[1]

                        elif geom_type == 'MultiLineString':
                            coords = np.array(coords)[0]
                            coords_x_min = np.min(coords[:, 0])
                            coords_y_min = np.min(coords[:, 1])
                            coords_x_max = np.max(coords[:, 0])
                            coords_y_max = np.max(coords[:, 1])

                        elif geom_type == 'MultiPolygon':

                            if len(coords[0]) > 1:
                                # The first array is the outer polygon; the following ones are 'holes' inside the former.
                                coords = np.array(coords[0][0])

                                coords_x_min = np.min(coords[:, 0])
                                coords_y_min = np.min(coords[:, 1])
                                coords_x_max = np.max(coords[:, 0])
                                coords_y_max = np.max(coords[:, 1])

                            else:
                                coords = np.array(coords)[0, 0]

                                coords_x_min = np.min(coords[:, 0])
                                coords_y_min = np.min(coords[:, 1])
                                coords_x_max = np.max(coords[:, 0])
                                coords_y_max = np.max(coords[:, 1])

                        else:
                            raise TypeError("Expected types are MultiLineString, MultiPolygon and Point. Got {} instead.".format(geom_type))

                        rects = get_rects(coords_x_min, coords_y_min, coords_x_max, coords_y_max, H, W)

                        feature['properties'] = OrderedDict(fid_zbg=feature['properties']['fid_zbg'])

                        for rect in rects:
                            data[rect][geom_type].append(feature)

                        c += 1

                        if c >= RAM_LIMIT:
                            print("Data taking a lot of RAM. Preemptively relieving (this is not a bad thing, it is bound to happen a few times).")

                            write_to_file(data, fn_preposition, category_str)

                            del data
                            data = defaultdict(lambda: defaultdict(list))
                            c = 0
            except ValueError:
                pass

        print("Writing into files.")
        write_to_file(data,fn_preposition,category_str)


def write_to_file(data,fn_preposition,category_str):
    for rect,features_by_geom_dict in tqdm(data.items()):
        out_fn = os.environ['HOME'] + '/.ros/cache/cuzk_tools/topography/gpkg_files/'  + fn_preposition + '_' \
                                                        + str(rect[0])[1:] + '_' \
                                                        + str(rect[1])[1:] + '_' \
                                                        + str(rect[2])[1:] + '_' \
                                                        + str(rect[3])[1:]
        
        for geom_type in features_by_geom_dict.keys():  
            features = features_by_geom_dict[geom_type]

            with fiona.open(out_fn, mode="a", driver='GPKG', layer = get_layer_name(category_str, geom_type),
                            schema=get_simple_schema(geom_type), crs=get_simple_crs()) as out_layer:
                out_layer.writerecords(features)
            


if __name__ == "__main__":
    gpkg_file_path = os.environ['HOME'] + "/.ros/cache/cuzk_tools/topography/data.gpkg"
    #get_layer_schemas(gpkg_file_path)
    split_gpkg_into_files(gpkg_file_path, 'topography', USE_ORIGINAL_CATEGORIES)

    #visualize_layer(gpkg_file_path,'BudovaJednotlivaNeboBlokBudov')