load('AcfCaltech+Detector.mat');
opts = detector.opts;
pPyramid = opts.pPyramid;
fp = fopen('acfmodel_caltech.bin', 'w');

%% pPyramid.pChns
fwrite(fp, int32(pPyramid.pChns.shrink), 'int32');
fwrite(fp, single(pPyramid.pChns.pColor.smooth), 'single');

fwrite(fp, int32(pPyramid.pChns.pGradMag.colorChn), 'int32');
fwrite(fp, single(pPyramid.pChns.pGradMag.normRad), 'single');
fwrite(fp, single(pPyramid.pChns.pGradMag.normConst), 'single');
fwrite(fp, logical(pPyramid.pChns.pGradMag.full), 'logical');

fwrite(fp, int32(pPyramid.pChns.pGradHist.nOrients), 'int32');
fwrite(fp, logical(pPyramid.pChns.pGradHist.softBin), 'logical');
fwrite(fp, logical(pPyramid.pChns.pGradHist.useHog), 'logical');
fwrite(fp, single(pPyramid.pChns.pGradHist.clipHog), 'single');

%% pPyramid
fwrite(fp, int32(pPyramid.nPerOct), 'int32');
fwrite(fp, int32(pPyramid.nOctUp), 'int32');
fwrite(fp, int32(pPyramid.nApprox), 'int32');
fwrite(fp, uint32(3), 'uint32');
fwrite(fp, single(pPyramid.lambdas), 'single');
fwrite(fp, int32(pPyramid.pad), 'int32');
fwrite(fp, single(pPyramid.minDs), 'single');
fwrite(fp, single(pPyramid.smooth), 'single');
fwrite(fp, logical(pPyramid.complete), 'logical');

%% filters
rows = uint32(size(opts.filters, 1));
cols = uint32(size(opts.filters, 2));
channels = uint32(size(opts.filters, 3) * size(opts.filters, 4));
fwrite(fp, rows, 'uint32');
fwrite(fp, cols, 'uint32');
fwrite(fp, channels, 'uint32');
fwrite(fp, opts.filters, 'single');

%% modelDs, modelDsPad
fwrite(fp, single(opts.modelDs), 'single');
fwrite(fp, single(opts.modelDsPad), 'single');

%% other parameters
fwrite(fp, uint32(opts.stride), 'uint32');
fwrite(fp, single(opts.cascThr), 'single');
fwrite(fp, single(opts.cascCal), 'single');

%% Forest
clf=detector.clf;
nTreeNodes = uint32(size(clf.fids, 1));
nTrees = uint32(size(clf.fids, 2));
fwrite(fp, nTreeNodes, 'uint32');
fwrite(fp, nTrees, 'uint32');
fwrite(fp, clf.treeDepth, 'uint32');
fwrite(fp, clf.fids, 'uint32');
fwrite(fp, clf.thrs, 'single');
fwrite(fp, clf.child, 'uint32');
fwrite(fp, clf.hs, 'single');
fclose(fp);
