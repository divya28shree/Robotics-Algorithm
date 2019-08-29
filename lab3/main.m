imageList = {"image1"; "image2"; "image3"; "image4"; "image5"; "image6"}
for entry = 1:6
  fileName = strcat(imageList{entry,1},".jpg");
   img = imread(fileName);
   [x y] = size(img);
   s=floor(x/3);
   green = img(1:s, 1:y);
   blue = img(s+1:s*2, 1:y);
   red = img(s*2+1:s*3, 1:y);
   newImg=cat(3,red,blue,green);
   newImgFile= strcat(imageList{entry,1},"-color.jpg");
   imwrite(newImg,newImgFile);
   #Calling SSD
   im_align1(green, blue, red,imageList{entry,1});
   im_align2(green, blue, red,imageList{entry,1});
endfor;