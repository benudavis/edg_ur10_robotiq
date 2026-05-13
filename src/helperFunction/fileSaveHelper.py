#!/usr/bin/env python
import os
from datetime import datetime
import numpy as np
import pandas as pd
from scipy.io import savemat
import re
import cv2

class fileSaveHelp(object):
    def __init__(self, savingFolderName='EDG_Experiment', fileName=None, saveFrames=True):
        self.savingFolderName = savingFolderName
        self.fileName = fileName
        self.saveFrames = saveFrames
        self.ResultSavingDirectory = os.path.expanduser('~') + '/' + self.savingFolderName + '/' + datetime.now().strftime("%y%m%d")
        if not os.path.exists(self.ResultSavingDirectory):
            os.makedirs(self.ResultSavingDirectory)

    def getLastMatFileSaved(self):
        fileList = []
        for file in os.listdir(self.ResultSavingDirectory):
            if file.endswith(".mat"):
                fileList.append(file)
        try:
            return sorted(fileList)[-1]
        except Exception as e:
            print(e)
        return "none"

    def clearTmpFolder(self):
        fileList = []
        for file in os.listdir("/tmp"):
            if file.endswith(".csv"):
                fileList.append(os.path.join("/tmp", file))
        for fileName in fileList:
            os.remove(fileName)

    def saveDataParams(self, args=None, appendTxt='', image_frames=None):
        print("[DEBUG] saveDataParams called with image_frames")

        tmp_dummyFolder = '/tmp/processed_csv'
        if not os.path.exists(tmp_dummyFolder):
            os.makedirs(tmp_dummyFolder)

        fileList = []
        for file in os.listdir("/tmp"):
            if file.endswith(".csv"):
                fileList.append(os.path.join("/tmp", file))

        print("csv files: ", fileList)
        print("grabbing columns from csv files into one dataframe")
        savingDictionary = {}
        errorCount = 0
        for fileName in fileList:
            print("trying file: ", fileName)
            try:
                df = pd.read_csv(fileName)
                thisColumnName = df.columns.tolist()
                splitedList = re.split('_|\.', fileName)
                thisTopicName = ''.join(splitedList[4:-1])
                savingDictionary[thisTopicName + "_columnName"] = thisColumnName
                savingDictionary[thisTopicName + "_data"] = df.values
                os.rename(fileName, tmp_dummyFolder + '/' + os.path.basename(fileName))
            except Exception as e:
                print(e)
                errorCount += 1

        if errorCount > 0:
            print("!!!!-- Missed ", errorCount, " csv files --!!!!")

        if args is not None:
            argsDic = vars(args)
            for key in list(argsDic.keys()):
                savingDictionary[key] = argsDic[key]

        # savingFileName_noDir = 'DataLog_' + '_'.join(splitedList[1:4])
        from datetime import datetime
        time_tag = datetime.now().strftime("%Y_%m%d_%H%M%S")
        if self.fileName is None:
            savingFileName_noDir = 'DataLog_' + time_tag
        else:
            savingFileName_noDir = self.fileName

        savingFileName = self.ResultSavingDirectory + '/' + savingFileName_noDir + '.mat'
        print(savingFileName)

        # Save image frames with timestamps
        if image_frames is not None and isinstance(image_frames, dict):
            try:
                if (self.saveFrames):
                    if 'images' in image_frames and 'timestamps' in image_frames:
                        stream_name = 'digit'
                        streams = {stream_name: image_frames}
                    else:
                        streams = image_frames

                    for stream_name, stream_frames in streams.items():
                        if not isinstance(stream_frames, dict):
                            continue
                        if 'images' not in stream_frames or 'timestamps' not in stream_frames:
                            continue
                        if len(stream_frames['images']) == 0:
                            continue

                        image_array = np.stack(stream_frames['images'], axis=0)
                        timestamps = np.array(stream_frames['timestamps'])
                        safe_stream_name = str(stream_name).strip('/').replace('/', '_')

                        savingDictionary[f"{safe_stream_name}_image_frames"] = image_array
                        savingDictionary[f"{safe_stream_name}_image_timestamps"] = timestamps
                        print(f"[✓] Saved image frames and timestamps for {safe_stream_name} to .mat")

                        video_filename = savingFileName.replace('.mat', f'_{safe_stream_name}.avi')
                        height, width = image_array.shape[1], image_array.shape[2]
                        if image_array.ndim == 4:
                            _, height, width, _ = image_array.shape

                        fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        out = cv2.VideoWriter(video_filename, fourcc, 30.0, (width, height))
                        for frame in image_array:
                            if frame.ndim == 2:
                                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                            else:
                                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                            out.write(frame)
                        out.release()
                        print(f"[✓] Saved video: {video_filename}")
                else:
                    # calculate delta intensity wrt first frame
                    first_frame = image_frames['images'][0]
                    delta_frames = np.array([np.abs(frame.astype(np.float32) - first_frame.astype(np.float32)) for frame in image_frames['images']])
                    timestamps = np.array(image_frames['timestamps'])
                    delta_intensity = np.mean(delta_frames, axis=(1,2,3))

                    # we can just save delta intensity
                    savingDictionary["digit_change_intensity"] = delta_intensity
                    savingDictionary["digit_image_timestamps"] = timestamps
                    print("[✓] Saved delta intensity of image frames to .mat")

            except Exception as e:
                print(f"[!] Failed to save DIGIT video or frames: {e}")
        
        else:
            print("[!] No image frames to save.")

        savemat(savingFileName, savingDictionary)
        print("savingFileName_noDir: ", savingFileName_noDir)
        return self.ResultSavingDirectory, savingFileName_noDir
