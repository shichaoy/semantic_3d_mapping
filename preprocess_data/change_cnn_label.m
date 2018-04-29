% Map DilationNet label defination to our CRF3d label definition.

clear;
data_root='';

cnn_to_ours_conversion=[1,4,2,6,9,3,7,10,11,5,8];  % old 1-11  to our new 1-11
num_labels=11;

num_img = length(dir([data_root 'label_2d_bin/' '*.bin']));

for frame_id=1:numimg  % I will minus -1
    fprintf('processing %d out of %d \n',frame_id,500);
    
    %% change label image
    raw_cnn_img_file=[data_root 'label_2d_img_raw/' sprintf('%06d',frame_id-1) '_bw.png'];  % raw output from dilanetCNN.
    new_cnn_img_file=[data_root 'label_2d_img/' sprintf('%06d',frame_id-1) '_bw.png'];
    raw_cnn_img = imread(raw_cnn_img_file);  % start from 0 - 10
    raw_cnn_img = raw_cnn_img+1; % 1-11
    for row=1:size(raw_cnn_img,1)
        for col = 1:size(raw_cnn_img,2)            
                raw_cnn_img(row,col)=cnn_to_ours_conversion(raw_cnn_img(row,col))-1;  % change to 0-10
        end
    end    
%     figure();imagesc(raw_cnn_img)
    imwrite(uint8(raw_cnn_img),new_cnn_img_file);
    
    
    img_height = size(raw_cnn_img,1);
    img_width = size(raw_cnn_img,2);
    
    %% change label binary file
    raw_cnn_bin_file=[data_root 'label_2d_bin_raw/' sprintf('%06d',frame_id-1) '.bin'];  % all pixels' (row order) label distribution
    new_cnn_bin_file=[data_root 'label_2d_bin/' sprintf('%06d',frame_id-1) '.bin'];
    
    fd = fopen(raw_cnn_bin_file,'r');
    U = fread(fd,img_height*img_width*num_labels,'single');  % single  for float   double.
    fclose(fd);
    prob=reshape(U,num_labels,img_height*img_width)'; 

    % visualize CNN binary prediction.
%     [~,label_pixs]=max(prob,[],2);
%     label_map=reshape(label_pixs,img_width,img_height)';
%     figure();imagesc(label_map)

    
    newprob=zeros(size(prob));
    for class=1:num_labels
        newprob(:,cnn_to_ours_conversion(class))=prob(:,class);
    end
    fileID = fopen(new_cnn_bin_file,'w');
    fwrite(fileID,single(newprob)','single');  % matlab save data column by column. so need transpose!
    fclose(fileID);
    
end
