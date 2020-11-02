from PIL import Image, ImageFilter 

im = Image.open("output1.jpg")
# Converting the image to greyscale, as Sobel Operator requires 
# input image to be of mode Greyscale (L) 
img = im.convert("L") 
  
# Calculating Edges using the passed laplican Kernel 
final = img.filter(ImageFilter.Kernel((3, 3), (1, 0, -1, 2, 0, -2, 1, 0, -1), 1, 0)) 

final1 = img.filter(ImageFilter.Kernel((3, 3), (1, 2, 1, 0, 0, 0, -1, -2, -1), 1, 0)) 
  
final.show()
final1.show()