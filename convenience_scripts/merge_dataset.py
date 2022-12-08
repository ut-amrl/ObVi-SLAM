import os
import argparse
import shutil
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--config',    default="configs/iv_bbox/merge_dataset.yaml",     help="")

'''
helps referencing object in a dictionary as dict.key instead of dict['key']
'''
class Namespace(object):
    def __init__(self, adict):
        self.__dict__.update(adict)

def prepare_args(filename):
    args = Namespace(yaml.safe_load(open(filename)))
    return args

def merge(args):
    uid = 0

    outdir = os.path.join(args.directory, args.out_name)
    if os.path.exists(outdir):
        shutil.rmtree(outdir)
    os.mkdir(outdir)
    outbbox_dir = os.path.join(outdir, "bboxes")
    outgt_dir   = os.path.join(outdir, "gts")
    outimg_dir  = os.path.join(outdir, "images")
    os.mkdir(outbbox_dir); os.mkdir(outgt_dir); os.mkdir(outimg_dir)

    for in_name in args.in_names:
        indir = os.path.join(args.directory, in_name)
        inbbox_dir = os.path.join(indir, "bboxes")
        ingt_dir   = os.path.join(indir, "gts")
        inimg_dir  = os.path.join(indir, "images")
        for filename in os.listdir(inbbox_dir):
            filename = filename.split('.')[0]

            inbbox = os.path.join(inbbox_dir, filename + args.bbox_format)
            ingt   = os.path.join(ingt_dir,   filename + args.bbox_format)
            inimg  = os.path.join(inimg_dir,  filename + args.img_format)
            outbbox = os.path.join(outbbox_dir, str(uid) + args.bbox_format)
            outgt   = os.path.join(outgt_dir,   str(uid) + args.bbox_format)
            outimg  = os.path.join(outimg_dir,  str(uid) + args.img_format)
            
            shutil.copyfile(inbbox, outbbox); shutil.copyfile(ingt, outgt); shutil.copyfile(inimg, outimg)
            uid += 1
    return uid


if __name__ == '__main__':
    args = parser.parse_args()
    args = prepare_args(args.config)
    uid = merge(args)
    print("final uid: ", uid)