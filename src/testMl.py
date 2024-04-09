from keras.models import load_model 
import os

def deepVineyardModel(pathModel):
	model = load_model(pathModel,compile=False)
	return model

real_path = os.path.dirname(os.path.realpath(__file__)) ##path of directory with python script
model=deepVineyardModel(os.path.join(real_path,'mobileNetv3_segmentation_new1.h5'))
model.summary()