from scapy.utils import RawPcapReader
from scapy.layers.l2 import Ether
from scapy.layers.inet import IP, TCP
import struct
import pickle
import numpy as np
import cv2
import os
from colormaps import ColorMap

import socket

from sonarDisplay import warpSonar

if not os.path.exists("oculus_h.pkl"):
    import bpStructs as bpcs
    bpcs.saveStructs2Pkl()
    bpStructs = bpcs.getBpStruct()
else:
    with open("oculus_h.pkl", 'rb') as fid:
        bpStructs = pickle.load(fid)


# fileName = r'wiresharkRec/oculus.pcapng'
fileName = r'wiresharkRec/oculus_dynamic_change_range.pcapng'
# fileName = r'wiresharkRec/oculus_512beams_rangeChange.pcapng'
# fileName = r'wiresharkRec/oculus_noChange.pcapng'
with open("oculus_h.pkl", 'rb') as fid:
    bpStructs = pickle.load(fid)


def findStruct(payload, msgId):
    targetSize = len(payload)
    ret = None
    for structName in bpStructs['structs'].keys():
        print(targetSize, bpStructs['structs'][structName]['sizeof'])
        if targetSize == bpStructs['structs'][structName]['sizeof']:
            print(structName)
            ret = bpStructs['structs'][structName]
            break
    return ret



def parseStruct(payload, st, packStr = None):
    if packStr is None:
        packStr = st['packString']

    tmp = struct.unpack(packStr, payload)
    data = {}

    for idx, key in enumerate(st["attributes"].keys()):
        data[key] = tmp[idx]
    
    return data

def parseSimpleFire2(payload):
    headerSize = bpStructs['structs']['OculusMessageHeader']['sizeof']
    header = parseStruct(payload[:headerSize], bpStructs['structs']['OculusMessageHeader'])
    ret = None
    if header['msgId'] in bpStructs['enums']['OculusMessageType']['revFields'].keys():
        curMsg = bpStructs['enums']['OculusMessageType']['revFields'][header['msgId']]
        if curMsg == 'messageDummy': #0xff
            print('Dummy Message')
        else:
            print('msg...')
            #import ipdb; ipdb.set_trace()

    elif header['msgId'] == 0x80:
        print(payload)    
    else:
        import ipdb; ipdb.set_trace()
    return ret


def getStructFields(st):
    import ipdb; ipdb.set_trace()
    packStr = ''
    kkkk = []
    msgLen = 0
    for key in st['attributes'].keys():
        if st['attributes'][key]['type'] != "struct":
            #print('==', key, st['attributes'][key]['type'])
            packStr += st['attributes'][key]['packStr']
            #print(st['attributes'][key]['sizeof'], st['attributes'][key]['packStr'])
            msgLen += st['attributes'][key]['sizeof']

            if st['attributes'][key]['type'] == 'arr':
                for i in range(len(st['attributes'][key]['packStr'])):
                    kk = key + '_%d_'%i
                    kkkk.append(kk)
            else:
                kkkk.append(key)
        elif st['attributes'][key]['type'] == "struct":
            subSt = bpStructs['structs'][st['attributes'][key]['strucType']]
            p, k, l = getStructFields(subSt)
            #print('--1->', packStr, keys)
            packStr += p
            kkkk += k
            msgLen += l
            #print('--2->', packStr, keys)
        
    return packStr, kkkk, msgLen




def unpack(pl, packStr, keys):
    ret = {}
    tmp = struct.unpack(packStr, pl)
    for idx, key in enumerate(keys):
        ret[key] = tmp[idx]
    
    return ret


def structParseByHeader(payload):
    headerSize = bpStructs['structs']['OculusMessageHeader']['sizeof']
    header = parseStruct(payload[:headerSize], bpStructs['structs']['OculusMessageHeader'])
    
    #print('--->>', len(payload))
    ret = None
    #print('cur msgId =', header['msgId'])
    if header['msgId'] in bpStructs['enums']['OculusMessageType']['revFields'].keys():
        curMsg = bpStructs['enums']['OculusMessageType']['revFields'][header['msgId']]
        if curMsg == "messageUserConfig":
            # not relevant... for parsing data (pc->sonar case...)
            payload = payload[headerSize: ]
            ret = parseStruct(payload, bpStructs['structs']["OculusUserConfig"])
            ret.update({'structName':"OculusUserConfig"})
            

        elif curMsg == "messageSimpleFire":
            st = bpStructs['structs']["OculusSimpleFireMessage2"]
            '''
            payload = payload[headerSize: ]
            packString = '<'
            for key in st['attributes'].keys():
                if key != "head":
                    packString += st['attributes'][key]['packStr']
                    #print(key, st['attributes'][key]['packStr'])
            '''
            ret = parseStruct(payload, st) #, packStr=packString)
            print('------> send flags:', bin(np.uint8(ret['flags'])))
            ret.update({'structName':"OculusSimpleFireMessage2"})
            print('---1---') 
            #import ipdb; ipdb.set_trace()
            

        elif curMsg == "messageSimplePingResult":
            st = bpStructs['structs']["OculusSimplePingResult2"]

            ret = parseStruct(payload[:st['sizeof']], st)
            
            ret.update({'structName':"OculusSimplePingResult2", 'plUsed':st['sizeof']})
            print('------> recieved flags:', bin(np.uint8(ret['flags'])))
            print('---2---') 
            #import ipdb; ipdb.set_trace()
        elif curMsg == "messagePingResult":
            pass

        elif curMsg == 'messageDummy': #0xff
            pass ##print('Dummy Message')

        else:
            print( '--> ', header['msgId'] )
            print( '---> ', curMsg )
            #import ipdb; ipdb.set_trace()
        
        print('<>%s<>'%curMsg, ret)

    elif header['msgId'] == 0x80:
        # user data
        data = payload[16:].decode()
        for line in data.strip().split('\n'):
            print(line)
    else:
        pass
        ##print('unknown msg...')
        #import ipdb; ipdb.set_trace()

    return ret




# def color_map_image(image, color_map):
#     # Allocate space for the color image.
#     color_image = np.empty((image.shape[0], image.shape[1], 3), dtype=np.uint8)

#     # Map each pixel to its color.
#     for r in range(image.shape[0]):
#         for c in range(image.shape[1]):
#             color_image[r, c] = color_map.lookup(image[r, c])

#     return color_image

def color_map_image(image, color_map):
    # color_image = color_map.data['_inferno_data_uint8'][image]
    color_image = color_map.data['_inferno_data_float'][image]

    
    # color_image = color_image.astype(np.float32)
    # color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    return color_image


# # slow
# def cartesian_to_polar(metadata, sonar_data):
#     n_ranges = metadata['nRanges']
#     azimuth_bounds = np.deg2rad(metadata['beamsDeg'])
#     minus_width = int(np.floor(n_ranges * np.sin(azimuth_bounds[0])))
#     plus_width = int(np.ceil(n_ranges * np.sin(azimuth_bounds[-1])))
#     width = plus_width - minus_width
#     origin_x = abs(minus_width)
#     new_image = np.zeros(shape=(n_ranges, width), dtype='uint8')

#     db = (azimuth_bounds[-1] - azimuth_bounds[0]) / metadata['nBeams'] if metadata['nBeams'] > 0 else 1
#     for x in range(new_image.shape[1]):
#         for y in range(new_image.shape[0]):
#             dx = x - origin_x
#             dy = new_image.shape[0] - y
#             range_val = np.sqrt(dx * dx + dy * dy)
#             azimuth = np.arctan2(dx, dy)
#             # xp = int(range_val)
#             xp = range_val
#             yp = int((azimuth - azimuth_bounds[0]) / db)
#             if xp >= 0 and xp < sonar_data.shape[0] and yp >= 0 and yp < sonar_data.shape[1]:
#                 new_image[y, x] = sonar_data[int(xp), int(yp)]
#     return new_image




# Fast
def cartesian_to_polar(metadata, sonar_data):
    n_ranges = metadata['nRanges']
    azimuth_bounds = np.deg2rad(metadata['beamsDeg'])
    minus_width = int(np.floor(n_ranges * np.sin(azimuth_bounds[0])))
    plus_width = int(np.ceil(n_ranges * np.sin(azimuth_bounds[-1])))
    width = plus_width - minus_width
    origin_x = abs(minus_width)

    db = (azimuth_bounds[-1] - azimuth_bounds[0]) / metadata['nBeams'] if metadata['nBeams'] > 0 else 1
    dy, dx = np.indices((n_ranges, width))
    dx -= origin_x
    dy = n_ranges - dy

    range_val = np.sqrt(dx * dx + dy * dy).astype('int')
    azimuth = np.arctan2(dx, dy)
    yp = ((azimuth - azimuth_bounds[0]) / db).astype('int')

    mask = (range_val >= 0) & (range_val < sonar_data.shape[0]) & (yp >= 0) & (yp < sonar_data.shape[1])
    new_image = np.zeros(shape=(n_ranges, width), dtype='uint8')
    new_image[dy[mask], dx[mask] + origin_x] = sonar_data[range_val[mask], yp[mask]]

    return new_image




cv2.namedWindow('aa', 0)
cv2.namedWindow('bb', 0)
# cv2.namedWindow('cc', 0)
def process_pcap(file_name):
    print('Opening {}...'.format(file_name))

    client_1 = '169.254.70.88:54375'
    client_2 = '169.254.99.25:54375'
    server = '169.254.70.16:52100'

    (client_1_ip, client_port) = client_1.split(':')
    (client_2_ip, client_port) = client_2.split(':')
    (server_ip, server_port) = server.split(':')
    
    count = 0
    interesting_packet_count = 0
    requested = ''

    sumData = 0
    imData = b''
    imStarted = False
    imSize = -1
    offset = 0
    w = 256
    h = 199
    mSize = 999999
    metadata = None

    ws = warpSonar()
    colormapper = ColorMap()
    
    for (pkt_data, pkt_metadata,) in RawPcapReader(file_name):
        count += 1
        
        ether_pkt = Ether(pkt_data)
        if 'type' not in ether_pkt.fields:
            # LLC frames will have 'len' instead of 'type'.
            # We disregard those
            continue

        if ether_pkt.type != 0x0800:
            # disregard non-IPv4 packets
            continue

        ip_pkt = ether_pkt[IP]
        
        if ip_pkt.proto != 6:
            # (?)Ignore non-TCP packet
            data = ip_pkt.payload['Raw'].fields['load']
            statusSize = bpStructs['structs']['OculusStatusMsg']['sizeof']
            stStatus   = bpStructs['structs']['OculusStatusMsg']
            if len(data) == 142:
                status = parseStruct(data, stStatus)

                print( socket.inet_ntoa(struct.pack("<I", status['ipMask'])) )
                print( socket.inet_ntoa(struct.pack("<I", status['ipAddr'])) )
                '''
                pos = 16+12+24

                tmp = struct.unpack("<"+"I"*3+"B"*6+"d"*9,data[pos:pos+(3*4+6+9*8)])
                print('#2', socket.inet_ntoa(struct.pack("I", tmp[0])) )
                print('#2', socket.inet_ntoa(struct.pack("I", tmp[1])) )
                '''
            continue

        payloadLen = ip_pkt.len - (ip_pkt.ihl * 4) - (ip_pkt[TCP].dataofs * 4)

        if ((ip_pkt.src == client_1_ip) or (ip_pkt.src == client_2_ip)) and (ip_pkt.dst == server_ip):
            ## PC -> Sonar
            if payloadLen > 0:
                print('-start'*10, '--- PC->Sonar')
                payload = ip_pkt.payload['Raw'].fields['load']
                ##print('-req->'*14, len(payload)) #, payload)
                request = structParseByHeader(payload)

                #import ipdb; ipdb.set_trace()
                print('!!<>!!<>'*5, request)
                print('-end'*10, '--- PC->Sonar')
                #import ipdb; ipdb.set_trace()
            else:
                #sync msgs (?)
                ##print('empty REQ')
                continue
            
            
        elif (ip_pkt.src == server_ip) and ((ip_pkt.dst == client_1_ip) or (ip_pkt.dst == client_2_ip)):
            ## Sonar -> pc
            
            if payloadLen > 0:
                ##print('-start'*10, '--- Sonar->PC')
                payload = ip_pkt.payload['Raw'].fields['load']

                if imStarted:
                    #print('part of im...')  
                    imData += payload
                    sumData = len(imData)
                    ##print('sum Data =', sumData)
                    #if sumData > offset+w*h:
                    if sumData > mSize:
                        print('-->', offset, w, h)

                        img = np.frombuffer(imData[offset - 2*w:offset+w*h -2*w], dtype='uint8').reshape((h, w))
                        # polar = cartesian_to_polar(metadata, np.frombuffer(imData[offset:offset+w*h], dtype='uint8').reshape((h, w)))
                        polar = cartesian_to_polar(metadata, img)
                        
                        # inverted_img = np.flipud(img)
                        # polar_inverted = cartesian_to_polar(metadata, inverted_img)

                        img = color_map_image(img, colormapper)
                        polar = color_map_image(polar, colormapper)

                        
                        with open('inData.pkl', 'wb') as fid:
                            pickle.dump([metadata, img], fid)

                        cv2.imwrite('input.jpg', img)
                        cv2.imshow('aa', img);
                        # print('date', data)
                        print("image size", img.shape)
                        # showIm = ws.warpSonarImage(metadata, cropped)
                        cv2.imshow('bb', polar);
                        # cv2.imshow('cc', polar_inverted);
                        cv2.waitKey(1)
                        imStarted = False
                        imData = b''
                        sumData = 0
                        #import ipdb; ipdb.set_trace()
                
                ##print('-resp->', len(payload)) #, payload)
                data = structParseByHeader(payload)
                
                if data is not None:
                    if data['structName'] == "OculusSimplePingResult2":
                        print('start of im...')
                        w               = data['nBeams']
                        h               = data['nRanges']
                        offset          = data['imageOffset']
                        mSize           = data['messageSize']
                        metaDataSize    = data['plUsed']

                        pl = payload##[data['plUsed']:]

                        beamsDeg = np.frombuffer(pl[metaDataSize:metaDataSize+w*2], dtype=np.short)/100
                        #import ipdb; ipdb.set_trace()
                        imData += pl[data['nBeams']*2:]
                        sumData = len(imData)
                        imSize = data['imageSize']
                        imStarted = True

                        metadata = data
                        metadata.update({"beamsDeg":beamsDeg})
                        
                else:
                    pass
                    #print("getting image...")
                ##print('-end'*10, '--- Sonar->PC')
            else:
                #sync msgs (?)
                ##print('empty RESP')
                continue
            
        else:
            print('src IP: %s ---- dst IP: %s'%(ip_pkt.src, ip_pkt.dst))
            print('not in the game...')
        
        



        #tcp_pkt = ip_pkt[TCP]


process_pcap(fileName)

