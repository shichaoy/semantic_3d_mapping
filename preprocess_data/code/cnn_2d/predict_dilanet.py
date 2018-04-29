#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from __future__ import print_function, division
import argparse
import cv2
import json
# import numba
import numpy as np
from os.path import dirname, exists, join, splitext,isfile
import os
import sys
from path import Path

import util


caffe_root = 'your/path/to/caffe/'
sys.path.insert(0, caffe_root + 'python')

import caffe


class Dataset(object):
    def __init__(self, dataset_name):
        self.work_dir = dirname(__file__)
        info_path = join(self.work_dir, 'datasets', dataset_name + '.json')
        if not exists(info_path):
            raise IOError("Do not have information for dataset {}"
                          .format(dataset_name))
        with open(info_path, 'r') as fp:
            info = json.load(fp)
        self.palette = np.array(info['palette'], dtype=np.uint8)
        self.mean_pixel = np.array(info['mean'], dtype=np.float32)
        self.dilation = info['dilation']
        self.zoom = info['zoom']
        self.name = dataset_name
        self.model_name = 'dilation{}_{}'.format(self.dilation, self.name)
        self.model_path = join(self.work_dir, 'models',
                               self.model_name + '_deploy.prototxt')

    @property
    def pretrained_path(self):
        p = join(dirname(__file__), 'pretrained',
                 self.model_name + '.caffemodel')
        if not exists(p):
            download_path = join(self.work_dir, 'pretrained',
                                 'download_{}.sh'.format(self.name))
            raise IOError('Pleaes run {} to download the pretrained network '
                          'weights first'.format(download_path))
        return p


def predict(dataset_name, input_path, output_path):
    print dataset_name, input_path, output_path
    dataset = Dataset(dataset_name)    
    net = caffe.Net(dataset.model_path, dataset.pretrained_path, caffe.TEST)
    label_margin = 186

    input_dims = net.blobs['data'].shape
    batch_size, num_channels, input_height, input_width = input_dims
    caffe_in = np.zeros(input_dims, dtype=np.float32)
    image = cv2.imread(input_path, 1).astype(np.float32) - dataset.mean_pixel

    image_size = image.shape
    output_height = input_height - 2 * label_margin
    output_width = input_width - 2 * label_margin
    image = cv2.copyMakeBorder(image, label_margin, label_margin,
                               label_margin, label_margin,
                               cv2.BORDER_REFLECT_101)

    num_tiles_h = image_size[0] // output_height + \
                  (1 if image_size[0] % output_height else 0)
    num_tiles_w = image_size[1] // output_width + \
                  (1 if image_size[1] % output_width else 0)

    prediction = []
    for h in range(num_tiles_h):
        col_prediction = []
        for w in range(num_tiles_w):
            offset = [output_height * h,
                      output_width * w]
            tile = image[offset[0]:offset[0] + input_height,
                         offset[1]:offset[1] + input_width, :]
            margin = [0, input_height - tile.shape[0],
                      0, input_width - tile.shape[1]]
            tile = cv2.copyMakeBorder(tile, margin[0], margin[1],
                                      margin[2], margin[3],
                                      cv2.BORDER_REFLECT_101)
            caffe_in[0] = tile.transpose([2, 0, 1])
            out = net.forward_all(**{net.inputs[0]: caffe_in})
            prob = out['prob'][0]
            col_prediction.append(prob)
        # print('concat row')
        col_prediction = np.concatenate(col_prediction, axis=2)
        prediction.append(col_prediction)
    prob = np.concatenate(prediction, axis=1)
    if dataset.zoom > 1:
        prob = util.interp_map(prob, dataset.zoom, image_size[1], image_size[0])
    prediction = np.argmax(prob.transpose([1, 2, 0]), axis=2)
    color_image = dataset.palette[prediction.ravel()].reshape(image_size)
    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    print('Writing', output_path)
    cv2.imwrite(output_path, color_image)



def batch_predict_kitti(dataset_name):

    caffe.set_mode_gpu()
    caffe.set_device(0)

    img_folder="path/to/preprocess_data/raw_imgs/"
    img_list_file="namelist_small.txt"
    save_folder="path/to/preprocess_data/"
    if not os.path.exists(save_folder+"label_2d_img/"):
        os.makedirs(save_folder+"label_2d_img/")
    if not os.path.exists(save_folder+"label_2d_bin/"):
        os.makedirs(save_folder+"label_2d_bin/")

    test_img_list = np.loadtxt(img_folder+img_list_file,  str, comments=None, delimiter='\n')
    data_counts = len(test_img_list)
    print "To processing: ",data_counts
    print "First one: ",test_img_list[1]    

    dataset = Dataset(dataset_name)    
    net = caffe.Net(dataset.model_path, dataset.pretrained_path, caffe.TEST)
    label_margin = 186

    input_dims = net.blobs['data'].shape
    batch_size, num_channels, input_height, input_width = input_dims  # batch is now 1.
    print "Using batch size %d "%(batch_size)

    for img_id in xrange(len(test_img_list)):
        print "processing %d  out of %d "%(img_id,len(test_img_list))
        rgb_img_file=img_folder+"image_2/"+test_img_list[img_id]
        saved_color_img_file=save_folder+"label_2d_img/"+test_img_list[img_id].replace(".png","_color.png")
        saved_bw_img_file=save_folder+"label_2d_img/"+test_img_list[img_id].replace(".png","_bw.png")
        saved_bin_file=save_folder+"label_2d_bin/"+test_img_list[img_id].replace(".png",".bin")
        # print saved_bin_file
        caffe_in = np.zeros(input_dims, dtype=np.float32)
        image = cv2.imread(rgb_img_file, 1).astype(np.float32) - dataset.mean_pixel

        image_size = image.shape
        output_height = input_height - 2 * label_margin
        output_width = input_width - 2 * label_margin
        image = cv2.copyMakeBorder(image, label_margin, label_margin,
                                   label_margin, label_margin,
                                   cv2.BORDER_REFLECT_101)

        num_tiles_h = image_size[0] // output_height + \
                      (1 if image_size[0] % output_height else 0)
        num_tiles_w = image_size[1] // output_width + \
                      (1 if image_size[1] % output_width else 0)
        
        for h in range(num_tiles_h):
            col_prediction = []
            for w in range(num_tiles_w):
                offset = [output_height * h,
                          output_width * w]
                tile = image[offset[0]:offset[0] + input_height,
                             offset[1]:offset[1] + input_width, :]
                margin = [0, input_height - tile.shape[0],
                          0, input_width - tile.shape[1]]
                tile = cv2.copyMakeBorder(tile, margin[0], margin[1],
                                          margin[2], margin[3],
                                          cv2.BORDER_REFLECT_101)
                caffe_in[0] = tile.transpose([2, 0, 1])
                out = net.forward_all(**{net.inputs[0]: caffe_in})
                prob = out['prob'][0]
        if dataset.zoom > 1:
            prob = util.interp_map(prob, dataset.zoom, image_size[1], image_size[0])
        # print prob.shape  # (11, 370, 1226)
        # print prob.dtype  # float32
        
        prediction = np.argmax(prob.transpose([1, 2, 0]), axis=2)
        # print "prediction shape, ", prediction.shape
        color_image = dataset.palette[prediction.ravel()].reshape(image_size)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        # print('Writing', saved_color_img_file)
        cv2.imwrite(saved_color_img_file, color_image)
        
        cv2.imwrite(saved_bw_img_file,prediction.astype(np.uint8))  # save as label ID image

        # prob_temp=prob.transpose([1, 2, 0])  # 370*1226*11
        # print prob_temp.shape
        prob.transpose([1, 2, 0]).flatten().tofile(saved_bin_file)  # save as binary file  # each row store's all class probability for one pixel. pixel is ordered by row





def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', nargs='?',
                        choices=['pascal_voc', 'camvid', 'kitti', 'cityscapes'])
    parser.add_argument('input_path', nargs='?', default='',
                        help='Required path to input image')
    parser.add_argument('-o', '--output_path', default=None)
    parser.add_argument('--gpu', type=int, default=-1,
                        help='GPU ID to run CAFFE. '
                             'If -1 (default), CPU is used')
    args = parser.parse_args()
    if args.input_path == '':
        raise IOError('Error: No path to input image')
    if not exists(args.input_path):
        raise IOError("Error: Can't find input image " + args.input_path)
    if args.gpu >= 0:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu)
        print('Using GPU ', args.gpu)
    else:
        caffe.set_mode_cpu()
        print('Using CPU')
    if args.output_path is None:
        args.output_path = '{}_{}.png'.format(
                splitext(args.input_path)[0], args.dataset)    
    predict(args.dataset, args.input_path, args.output_path)


if __name__ == '__main__':
    # main()
    batch_predict_kitti("kitti")