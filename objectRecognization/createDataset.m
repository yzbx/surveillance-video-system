function createDataset(dataset)
% from folder create an feature dataset
% video*-obj*-frame*.png --findstr

root=dataset;
filelist=dir(root);
filelist={filelist.name};
filenum=length(filelist)-2;

label={};
fileNames={};
dataset=[];
datasetSize=0;
%         filenum=min(filenum,100);
maxDatasetSize=filenum;
for i=1:filenum
    str=filelist{i+2};
    imgPath=[root,'\',str];
    
    disp(i);
    disp(imgPath);
    
    feature=featureExtract(imgPath);
    if(isempty(feature))    %small picture.
        continue;
    end
    a=findstr('-',str);
    
    
    if(isempty(dataset))
        featureSize=length(feature);
        dataset=zeros(maxDatasetSize,length(feature)+1);
        fileNames{maxDatasetSize}='';
        label{maxDatasetSize}='';
    end
    
    if(featureSize~=length(feature))
        warning(imgPath);
        error('featureSize is different');
    end
    
    datasetSize=datasetSize+1;
    dataset(datasetSize,:)=[feature datasetSize];
    fileNames{datasetSize}=imgPath;
    label{datasetSize}=str(1:a(2)-1);
end

dataset=dataset(1:datasetSize,:);
fileNames=fileNames(1:datasetSize);
label=label(1:datasetSize);
[idx,~]=grp2idx(label);
label=idx;
save('objectSetRecognize.mat','dataset','fileNames','label');
end

