import matplotlib.pyplot as plt
import matplotlib.animation as animation
from PIL import Image as PILImage
import io
from IPython.display import clear_output

class AniMaker():
    def __init__(self):
        self.ims = []
    
    def set_snapshot(self, num=1):
        for i in range(num):
            buffer = io.BytesIO()        
            plt.savefig(buffer, format='png')
            buffer.seek(0)
            im = PILImage.open(buffer)
            self.ims.append(im.copy())
        clear_output(wait=True)
        print('image added: ', len(self.ims))
        return im
    
    def make_gif(self, filename):
        self.ims[0].save(filename, 
                         format='GIF',
                         append_images=self.ims[1:], 
                         save_all=True, duration=100, loop=0)
