function x = isrot (a, tol)

% Uso: isrot(R)
% verifica che la matrice R sia una matrice di rotazione
%      isrot(R)==0 matrice di rotazione
%      isrot(R)~=0 errore
%

[r c] = size(a);
x = 1;
if r ~= 3
   disp ('Matrix has not 3 rows')
   elseif c ~= 3
          disp ('Matrix has not 3 columns')
          elseif norm ( a * a' - eye(3) ) > tol
                 disp ('Matrix is not orthonormal, i.e. ||(R''R-I)|| > 1E-10')
                 disp(norm ( a * a' - eye(3) ))
                 disp('-----------------------')
                 elseif det (a) < 0
		  	disp ('Matrix determinant is -1')
			else x = 0;
end
