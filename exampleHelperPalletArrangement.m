function goalLoc = exampleHelperPalletArrangement(id,palletDim,palletLoc,boxDim)
%EXAMPLEHELPERPALLETARRANGEMENT computes the position of the current box on the
% pallet based on its location in the palletizing pattern.
%   boxDim is now [L W H] (1x3)

boxDimX = 1.1*boxDim(1); % expand footprint slightly to prevent collisions
boxDimY = 1.1*boxDim(2);
boxDimZ = boxDim(3);     % vertical stacking, no margin needed

nRows = floor(palletDim(1)/boxDimX);
nCols = floor(palletDim(2)/boxDimY);

cornerLocation = zeros(1,3);
cornerLocation(1) = palletLoc(1) + palletDim(1)/2;
cornerLocation(2) = palletLoc(2) + palletDim(2)/2;
cornerLocation(3) = palletLoc(3) + palletDim(3)/2;

nBoxesPerLayer = nRows*nCols;
layerID = ceil((id+1)/nBoxesPerLayer);
idInLayer = (id+1) - nBoxesPerLayer*(layerID - 1);

rowID =  ceil(idInLayer/nCols);
colID =  mod(idInLayer-1,nCols)+1;

goalLoc = zeros(1,3);
goalLoc(1) = cornerLocation(1) - (rowID-1)*boxDimX - boxDimX/2;
goalLoc(2) = cornerLocation(2) - (colID-1)*boxDimY - boxDimY/2;
goalLoc(3) = cornerLocation(3) + (layerID-1)*boxDimZ + boxDimZ/2;

goalLoc = double([goalLoc(1) goalLoc(2) goalLoc(3)]);
goalLoc = reshape(goalLoc,1,3);
end