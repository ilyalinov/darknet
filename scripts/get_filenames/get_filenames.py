import os

path = 'C:/Users/ilya_/Downloads/data_object_image_2/testing/image_2'
l = os.listdir(path)[0:50]

f = open("names.txt", "w")
f.writelines(["%s\n" %i for i in l])
f.close()