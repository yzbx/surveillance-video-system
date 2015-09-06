function [precision, recall] = unlimitedSVM(label,fileNames,numOfReturnedImgs, ...
    dataset, queryImageFeatureVector, metric)
%# load dataset and extract image names
img_names = dataset(:, end);
dataset(:, end) = [];

% extract image name from queryImageFeatureVector
query_img_name = queryImageFeatureVector(:, end);
queryImageFeatureVector(:, end) = [];

[g, gn] = grp2idx(label);                      %# nominal class to numeric

%# split training/testing sets
group=unique(g);
posIdxs=cell(length(group),1);
negIdxs=cell(length(group),1);
for i=1:length(group)
    pos=find(g==group(i));
    posIdxs{i}=pos;
    neg=find(g~=group(i));
    if(length(neg)>3*length(pos))
        idx=randperm(length(neg),3*length(pos));
        negIdxs{i}=neg(idx);
    else
        negIdxs{i}=neg;
    end
end


% [trainIdx, testIdx] = crossvalind('HoldOut', label, 1/2); % split the train and test labels 50%-50%
% pairwise = nchoosek(1:size(gn, 1), 2);            %# 1-vs-1 pairwise models
svmModel = cell(length(group), 1);            %# store binary-classifers
predTest = cell(numel(svmModel),1); %# store binary predictions
precision=zeros(length(group),1);
recall=zeros(length(group),1);
%# classify using one-against-one approach, SVM with 3rd degree poly kernel
for k=1:numel(svmModel)
    pos=posIdxs{k};
    neg=negIdxs{k};
    
    data=dataset([pos;neg],:);
    lab=zeros(length(pos)+length(neg),1);
    lab(1:length(pos))=1;
    
    [trainIdx, testIdx] = crossvalind('HoldOut', lab, 1/2); % split the train and test labels 50%-50%
    %# get only training instances belonging to this pair
%     idx = trainIdx & any( bsxfun(@eq, g, pairwise(k,:)) , 2 );
    idx=trainIdx;
    
    %# train
%     svmModel{k} = svmtrain(dataset(idx,:), g(idx), ...
%         'BoxConstraint',2e-1, 'Kernel_Function','polynomial', 'Polyorder',3);
    svmModel{k} = svmtrain(data(idx,:), lab(idx), ...
        'BoxConstraint', Inf, 'Kernel_Function', 'rbf', 'rbf_sigma', 14.51);

    %# test
    predTest{k} = svmclassify(svmModel{k}, data(testIdx,:)); % matlab native svm function
    
    TP=sum(and(lab(testIdx),predTest{k}));
    TN=sum(and(~lab(testIdx),~predTest{k}));
    FP=sum(and(~lab(testIdx),predTest{k}));
    FN=sum(and(lab(testIdx),~predTest{k}));
    precision(k)=(TP+TN)/(TP+TN+FP+FN);
    recall(k)=TP/(TP+FN);
    fprintf('SVM (1-against-(n-1)):\naccuracy = %.2f%%\n recall=%.2f%%', ...
        100*precision(k),100*recall(k));
end

precision=precision*100;
recall=recall*100;

% save('unlimitedSVMModel.mat','svmModel');
% pred = mode(predTest, 2);   %# voting: clasify as the class receiving most votes

%# performance
% cmat = confusionmat(g(testIdx), pred); %# g(testIdx) == targets, pred == outputs
% final_acc = 100*sum(diag(cmat))./sum(cmat(:));
% fprintf('SVM (1-against-1):\naccuracy = %.2f%%\n', final_acc);
% fprintf('Confusion Matrix:\n'), disp(cmat)
% assignin('base', 'cmatrix', cmat);



% % Precision and recall
% % 1st class
% precision = zeros(size(gn, 1), 1);
% recall = zeros(size(gn, 1), 1);
% 
% precision = cmat(1, 1)/sum(cmat(:, 1)); % tp/tp+fp, where tp = true positive, fp = false positive
% recall = cmat(1, 1)/sum(cmat(1, :)); % tp/tp+fn, where fn = false negatives
% % % 2nd class and forward
% 
% % unique(g)~! unique(g(testIdx))
% m=unique(g(testIdx));
% % for c = 2:size(gn, 1)
% for c=2:size(m,1)
%     precision(c) = cmat(c, c)/sum(cmat(c:end, c));
%     recall(c) = cmat(c, c)/sum(cmat(c, c:end));
% end
% 
% verify predictions
% dataset = [dataset img_names lbls];
% testData = dataset(testIdx, :);
% classesInTestData = sort(testData(:, end)); % 500 samples from 10 classes
% predictedImgs = dataset(pred, :);
% dataset(:, end) = [];
predQueryImg=zeros(numel(svmModel),1);

for k = 1:numel(svmModel)
    %# test
    predQueryImg(k) = svmclassify(svmModel{k}, queryImageFeatureVector); % queryImage = x.jpg, row=x from dataset
end
predFinalQueryImg = find(predQueryImg==1); % predicted final image in class x using voting

if(isempty(predFinalQueryImg))
    fprintf('do not find similar image');
else
    fprintf('Predicted Query Image Belongs to Class = %d\n', predFinalQueryImg(1));
    disp(predFinalQueryImg);
end

dataset = [dataset img_names];
for i=1:length(predFinalQueryImg)
    % take all images from dataset that belong to class x
    
    imgsInClassX = dataset(g == predFinalQueryImg(i), : );
    disp('find similar image class: ');
    disp(gn(predFinalQueryImg(i)));
    % Perform knn with queryImage and imgsInClassX
    imgsInClassXWithoutLbls = imgsInClassX;
%     imgsInClassXWithoutLbls(:, end) = [];
    % imgsInClassXWithoutLbls(:, end) = [];
    
    figure;
    L2(fileNames,numOfReturnedImgs, [queryImageFeatureVector query_img_name],...
        imgsInClassXWithoutLbls, metric);
end

end