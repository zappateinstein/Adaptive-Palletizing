function spawnBox(id,boxPosition,boxDim,boxWeight)
bosPosSim3D = [boxPosition(1), -boxPosition(2), boxPosition(3)];
World = sim3d.World.getWorld(bdroot);
if isempty(World.Root.findBy('ActorName',['collBox' num2str(id)],'first'))
    Box = sim3d.Actor('ActorName',['collBox' num2str(id)],'Mobility',sim3d.utils.MobilityTypes.Movable);
    add(World,Box)
    createShape(Box,'box', [boxDim(1),boxDim(2),boxDim(3)]);
    Box.Translation = bosPosSim3D;
    Box.Color = [200 100 16]/256;
    Box.Mass = boxWeight;
    %Box.Physics = true;
    Box.Collisions = true;
    Box.Parent = World.Root.findBy('ActorName','Warehouse','first');
end
end