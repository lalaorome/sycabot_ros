import pandas as pd

class recorder():
    def __init__(self):
        self.datas = []
        self.df = pd.DataFrame()
        self.recording = False
        self.run = 0

    def record(self, input, centroid_idx):
        self.datas.append([input,centroid_idx[0],centroid_idx[1]])
        print(self.df)

    def save_recording(self):
        
        if not self.recording and not self.df.shape[0] == 0:
            datas = pd.DataFrame(self.datas, columns=[f'u{self.run}', f'x{self.run}', f'y{self.run}'])
            self.df = pd.concat([self.df,datas], axis = 1)
            path = '~/Documents/sycabot_ros/datas_recording/recording.csv'
            self.df.to_csv(path, index = False)
            self.df = pd.DataFrame()
            print(f'Recording saved to {path}')
        elif not self.recording and self.df.shape[0] == 0:
            print('No data to store. Try to record something before.')
        else :
            print("Can't save the recording ... Try to stop it before.")
    
    def start_recording(self):
        if not self.recording and self.df.shape[0] == 0:
            self.recording = True
            self.run += 1
            print("Recording started...")
        else :
            print("You can't start a recording that has alredy been started.")

    def new_run(self):
        datas = pd.DataFrame(self.datas, columns=[f'u{self.run}', f'x{self.run}', f'y{self.run}'])
        self.df = pd.concat([self.df,datas], axis = 1)
        if self.recording :
            self.run += 1
            self.datas = []
            print('New run started...')
        else :
            print("No recording started. Try to start one before recording a new run.")
        
    def stop_resume_recording(self):
        if self.recording : 
            self.recording = False
            print('Recording paused.')
        elif not self.recording :
            self.recording = True
            print('Recording resumed.')
        else :
            print("No recording started. Try to start one before stopping it.")
    
    def reinit_recording(self):
        if self.recording :
            self.datas = []
        print(f"run{self.run} reinitialised.")