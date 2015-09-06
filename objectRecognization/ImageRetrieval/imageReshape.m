function imageReshape()
% reshape objects to 384 x 256 or 256 x 384 so can be use in ImageRetrieval

src='D:\Program\matlab\bgslibrary_mfc\dataset\objects';
des='D:\Program\matlab\ImageRetrieval\objects';
% layernum=3;
len1=384;
len2=256;
pathlist1=dir(src);
filenum1=length(pathlist1);
filenamelist1={pathlist1.name};

filenum1=min(filenum1,1000);
picnum=0;
i=3;
while picnum<1000
    imgsrcpath=[src,'\',filenamelist1{i}];
    imgdespath=[des,'\',int2str(picnum),'.png'];
%     imgdespath=[des,'\',filenamelist1{i}];
    i=i+1;
    imgsrc=imread(imgsrcpath);
    [width,height]=size(imgsrc);
    if(width<20||height<20)
        warning(['bad pic: ',imgsrcpath]);
    else
        if(width>height)
            %row=width=len1, col=height [row,col]=[width,height]
            imgdes=imresize(imgsrc,[len1,len2]);
            imwrite(imgdes,imgdespath);
        else
            imgdes=imresize(imgsrc,[len2,len1]);
            imwrite(imgdes,imgdespath);
        end
        picnum=picnum+1;
    end
end