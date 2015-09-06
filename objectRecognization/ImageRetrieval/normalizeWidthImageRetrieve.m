function normalizeWidthImageRetrieve()
% feature extract:  normalize width -> extract
% svm
% show

createDateset=false;
if(createDateset)
root='D:\Program\matlab\CVonlineDataset\101_ObjectCategories';
pathlist1=dir(root);
filenum1=length(pathlist1)-2;
filenamelist1={pathlist1.name};
filenum1=min(filenum1,10);

datasetSize=0;
maxDatasetSize=50*filenum1;
dataset=[];
label=[];
classNames{filenum1}='';
fileNames{maxDatasetSize}='';
featureSize=0;
%% feature extract
for i=1:filenum1
    src=[root,'\',filenamelist1{i+2}];
    disp(src);
    pathlist2=dir(src);
    filenum2=length(pathlist2)-2;
    filenamelist2={pathlist2.name};
    
    imgClassName=filenamelist1{i+2};
    classNames{i}=imgClassName;
    filenum2=min(filenum2,50);
    for j=1:filenum2
        imgPath=[src,'\',filenamelist2{j+2}];
        %       img=imread(imgPath);
        feature=featureExtract(imgPath);
        
        if(isempty(dataset))
            featureSize=length(feature);
            dataset=zeros(maxDatasetSize,length(feature)+1);
            label=zeros(maxDatasetSize,1);
        end
        
        if(featureSize~=length(feature))
            warning(imgPath);
            error('featureSize is different');
        end
        
        datasetSize=datasetSize+1;
        dataset(datasetSize,:)=[feature datasetSize];
        fileNames{datasetSize}=imgPath;
        label(datasetSize)=i;
    end
end

%% save dataset
label=label(1:datasetSize,:);
dataset=dataset(1:datasetSize,:);
save('yzbx.mat','dataset','label','fileNames');
else
    load yzbx.mat
end
%% SVM
    metric=5;
    queryFeature=dataset(201,:);
    
%     fileNames=fileNames{1:datasetSize};
    [precision, recall, cmat] = svm(label,fileNames,20, dataset, queryFeature, metric)
%%


    function feature=featureExtract(imgPath)
        %         imgInfo = imfinfo(imgPath);
        img=imread(imgPath);
        width=256;
        [w,h,c]=size(img);
        if(c==1)
           gray=img; 
        else
            gray=rgb2gray(img);
        end
        queryImage=imresize(gray,[width,ceil(h*width/w)]);
        
        grayHist = imhist(queryImage);
        grayHist = grayHist/sum(grayHist);
        grayHist = grayHist(:)';
        color_moments = [mean(mean(queryImage)) std(std(double(queryImage)))];
        [meanAmplitude, msEnergy] = gaborWavelet(queryImage, 4, 6); % 4 = number of scales, 6 = number of orientations
        wavelet_moments = waveletTransform(queryImage,'grayscale');
        % construct the queryImage feature vector
        feature = [grayHist color_moments meanAmplitude msEnergy wavelet_moments];
    end
end