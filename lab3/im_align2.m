function im_align2(blueImg, greenImg, redImg, fName)
  cr= normxcorr2(redImg,blueImg);
  [ypeak, xpeak] = find(cr==max(cr(:)));
  yoffSet = ypeak-size(blueImg,1);
  xoffSet = xpeak-size(blueImg,2);
  cg = normxcorr2(greenImg,blueImg);
  [ygpeak, xgpeak] = find(cg==max(cg(:)));
  ygoffSet = ygpeak-size(blueImg,1);
  xgoffSet = xgpeak-size(blueImg,2);
  nccIm = cat(3, circshift(redImg, [yoffSet xoffSet]), circshift(greenImg, [floor(ygoffSet/2) xgoffSet]), blueImg);
  fName=strcat(fName,"-ncc.jpg");
  imwrite(nccIm,fName);
  #f=fopen('myfile.txt', 'a');
   #fprintf(f, 'ImgNCC: : %.5f\n%.5f\n%.5f\n%.5f\n\n\n', yoffSet,xoffSet,floor(ygoffSet/2),xgoffSet);
   #fclose(f);
 endfunction
