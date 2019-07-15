# kinemator


It takes a pixel wise road segment, a pixel wise basis (vector of pixels) and pixelwise tracked objects and does following:
1. use pixel wise basis to convert pixel wise road segment into crosstrack road bands.
2. flatten pixelwise trackedobjects as if they are on 2D plane.
3. find the closest road segment centerline point on the image. Everything is eventually mapped onto this centerline. Travel of this centerline calculated using pixel wise basis.
4. get the travel (long distance) of this point on the centerline.
5. Use this travel to compute speed.
