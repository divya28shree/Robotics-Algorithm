function ssdVal=ssd(img1,img2)
  #disp("In function");
  sub=img1-img2;
  #disp(size(sub));
  sub=double(sub);
  ssdVal=0;
  [h w] = size(img1);
  x0=120;
  y0=100;
  ssdVal = sum(sum((img1(x0:h-x0, y0:w-y0) - img2(x0:h-x0, y0:w-y0)).^2));
  disp(ssdVal);
endfunction