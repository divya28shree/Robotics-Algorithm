function im_align1(blueImg, greenImg, redImg, fName)
  min1=999999999;
  min2=999999999;
  min1i=0; min1j=0;
  min2i=0; min2j=0;
  
  for i = -15:15
   for j = -15:15
     setImg1=circshift(redImg, [i j]);
     setImg2=circshift(greenImg,[i j]);
     ssd1=ssd(blueImg,setImg1);
     ssd2=ssd(blueImg,setImg2);
    if (double(ssd1)<min1)
      min1i=i;
      min1j=j;
      min1=double(ssd1); 
    endif
    if (double(ssd2)<min2)
      min2i=i;
      min2j=j;
      min2=double(ssd2); 
    endif        
   endfor
  endfor
    #f=fopen('myfile.txt', 'a');
   #fprintf(f, 'ImgSSD: : %.5f\n%.5f\n%.5f\n%.5f\n\n\n', min1i,min1j,min2i,min2j);
   #fclose(f);
 ssdImage = cat(3,circshift(redImg, [min1i min1j]), circshift(greenImg, [min2i, min2j]), blueImg);
 fName=strcat(fName,"-ssd.jpg");
 imwrite(ssdImage,fName);
endfunction
