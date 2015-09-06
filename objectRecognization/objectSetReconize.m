function objectSetRecognize(target,dataset)
% status=exist(dataset,'file');
% if(status~=7)   %a folder
%     load(dataset);
% else
%     createDataset(dataset);
%     load('objectSetRecognize.mat');
% end

%save('objectSetRecognize.mat','dataset','fileNames','label');
data=load(dataset);
fileNames=data.fileNames;
label=data.label;
dataset=data.dataset;

metric=5;
queryFeature=featureExtract(target);
queryFeature=[queryFeature 0];

%     fileNames=fileNames{1:datasetSize};
% [precision, recall, cmat] = svm(label,fileNames,20, dataset, queryFeature, metric)
[precision,recall]=unlimitedSVM(label,fileNames,20, dataset, queryFeature, metric)
query_img=imread(target);
subplot(3, 7, 1);
imshow(query_img, []);
title('Query Image', 'Color', [1 0 0]);

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
        
        if(w<20||h<20)
            feature=[];
        end
    end
end