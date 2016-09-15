load('modelBsds.mat');
opts = model.opts;
fp = fopen('model.bin', 'w');
fwrite(fp, int32(opts.imWidth), 'int32');
fwrite(fp, int32(opts.gtWidth), 'int32');
fwrite(fp, int32(opts.nTrees), 'int32');
fwrite(fp, int32(size(model.thrs, 1)), 'int32'); % nTreeNodes
%fwrite(fp, opts.fracFtrs, 'double');
fwrite(fp, int32(opts.nOrients), 'int32');
fwrite(fp, single(opts.grdSmooth), 'single');
fwrite(fp, single(opts.chnSmooth), 'single');
fwrite(fp, single(opts.simSmooth), 'single');
fwrite(fp, single(opts.normRad), 'single');
fwrite(fp, int32(opts.shrink), 'int32');
fwrite(fp, int32(opts.nCells), 'int32');
fwrite(fp, int32(opts.rgbd), 'int32');
fwrite(fp, int32(opts.stride), 'int32');
fwrite(fp, int32(opts.multiscale), 'int32');
fwrite(fp, int32(opts.sharpen), 'int32');
fwrite(fp, int32(opts.nTreesEval), 'int32');
fwrite(fp, int32(opts.nThreads), 'int32');
fwrite(fp, int32(opts.nms), 'int32');
fwrite(fp, int32(opts.nChns), 'int32');
fwrite(fp, int32(opts.nChnFtrs), 'int32');
fwrite(fp, int32(opts.nSimFtrs), 'int32');
fwrite(fp, model.thrs, 'single');
fwrite(fp, model.fids, 'uint32');
fwrite(fp, model.child, 'uint32');
fwrite(fp, model.segs, 'uint8');
fwrite(fp, model.nSegs, 'uint8');
fwrite(fp, int32(size(model.eBins, 1)), 'int32'); % sizeof eBins
fwrite(fp, model.eBins, 'uint16');
fwrite(fp, int32(size(model.eBnds, 1)), 'int32'); % sizeof eBnds
fwrite(fp, model.eBnds, 'uint32');
fclose(fp);
