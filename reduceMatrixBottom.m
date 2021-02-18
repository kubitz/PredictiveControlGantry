function reducedMatrix = reduceMatrixBottom(A,nRows,nCols)
reducedMatrix=A(1:size(A,1)-nRows,1:size(A,2)-nCols);
end

