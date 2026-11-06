buffer = cell(1, 10);

opts = detectImportOptions("boxes.xlsx", "VariableNamingRule", "preserve");
boxes = readtable("boxes.xlsx", opts);

boxes = table2struct(boxes);
while i <= lenght(boxes)
    for j = 1:10
      buffer{j} = boxes(i);
    end
end