function goalLoc = exampleHelperPalletArrangement(id,palletDim,palletLoc,boxDim)
%EXAMPLEHELPERPALLETARRANGEMENT computes the position of the current box on the
% pallet based its the location in the palletizing pattern. 

%   Copyright 2024 The MathWorks, Inc.

boxDimxy = 1.1*boxDim; % Expand the size of the box by a small amount to prevent collisions
nRows = floor(palletDim(1)/boxDimxy);
nCols = floor(palletDim(2)/boxDimxy);
  
% Determine the position of the starting corner on the pallet
cornerLocation = zeros(1,3);
cornerLocation(1) = palletLoc(1) + palletDim(1)/2;
cornerLocation(2) = palletLoc(2) + palletDim(2)/2;
cornerLocation(3) = palletLoc(3) + palletDim(3)/2;

% Compute the number of rows and columns that can fit on the pallet in one
% layer
nBoxesPerLayer = nRows*nCols;

% Determine which layer from the ground up the current box belongs to.
layerID = ceil((id+1)/nBoxesPerLayer);
idInLayer = (id+1) - nBoxesPerLayer*(layerID - 1);

% Compute the row and column IDs of that box in that layer
rowID =  ceil(idInLayer/nCols);
colID =  mod(idInLayer-1,nCols)+1;

% Convert this to position information based on box dimension
goalLoc = zeros(1,3);
goalLoc(1) = cornerLocation(1) - (rowID-1)*boxDimxy - boxDimxy/2;
goalLoc(2) = cornerLocation(2) - (colID-1)*boxDimxy - boxDimxy/2;
goalLoc(3) = cornerLocation(3) + (layerID-1)*boxDim + boxDim/2;

goalLoc = double([goalLoc(1) goalLoc(2) goalLoc(3)]);

end

