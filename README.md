# surveillance-video-system
a matlab surveillance video system

# 2015-09-04
# homepage
- [surviellance video system](http://blog.csdn.net/u010598445/article/details/47822195)

# foreground detection

# object tracking

# object recognization

# object classification

# event detection

# wonderful dataset used in project.
- [Multi-Camera Object Tracking Challenge](mct.idealtest.org/Datasets.html): fixed, non-overlapping videos.
- [Multiple Object Tracking Benchmark 2D MOT 2015](https://motchallenge.net): fixed and unfixed, picture file.
- [Visual Tracker Benchmark](cvlab.hanyang.ac.kr/tracker_benchmark):
---

# bug for new library, but this can make compile success.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/home/yzbx/linux/miniconda2/lib
```
$: pkg-config --libs bgsawk: symbol lookup error: /home/yzbx/linux/miniconda2/lib/libreadline.so.6: undefined symbol: PC
awk: symbol lookup error: /home/yzbx/linux/miniconda2/lib/libreadline.so.6: undefined symbol: PC
```
