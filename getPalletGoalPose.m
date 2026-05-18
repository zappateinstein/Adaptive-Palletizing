function bloc = getPalletGoalPose(ID, bDim)
%GETPALLETGOALPOSE get the  desired location of the box on the pallet

    bloc = double(zeros(3,1));
    
    persistent W pDimSLWorld pLocSLWorld bDimSLWorld
    if isempty(W)
        W = sim3d.World.getWorld(bdroot);
        
        % Find the pallet object
        Pallet = W.Root.findBy('ActorName','collPallet','first');
        pDim = max(Pallet.Vertices) - min(Pallet.Vertices);
        palletCenterLoc = (max(Pallet.Vertices) + min(Pallet.Vertices))/2;
        pLoc = Pallet.Translation + palletCenterLoc;
        pLocSLWorld = double([pLoc(1) -pLoc(2) pLoc(3)])';
        pDimSLWorld = double([pDim(1) pDim(2) pDim(3)])';
        bDimSLWorld = double([bDim bDim bDim])';
    end
    
    bLoc = exampleHelperPalletArrangement(pLocSLWorld,pDimSLWorld,bDimSLWorld,ID);
    bloc = double(bLoc)';
      
end