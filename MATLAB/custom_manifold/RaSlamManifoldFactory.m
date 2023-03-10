function M = RaSlamManifoldFactory(problem, lifted_dim)
    % unpack problem info
    num_poses = problem.num_poses;
    num_landmarks = problem.num_landmarks;
    num_ranges = problem.num_range_measurements;
    dim = problem.dim;

    % auxiliary useful info
    full_mat_len = num_poses*(dim+1) + num_landmarks + num_ranges;

    % indices of variables
    all_R_idxs = problem.all_R_idxs;
    all_t_idxs = problem.all_t_idxs;
    all_l_idxs = problem.all_l_idxs;
    all_d_idxs = problem.all_d_idxs;

    % create core product manifold and interface utilities
    elements = struct();
    elements.R = stiefelstackedfactory(num_poses, dim, lifted_dim);
    elements.t = euclideanfactory(     num_poses,    lifted_dim);

    if num_landmarks > 0
        elements.l = euclideanfactory(num_landmarks, lifted_dim);
    end
    if num_ranges > 0
        elements.d = obliquefactory(lifted_dim, num_ranges);
    end
    M_ = productmanifold(elements);

    M.toStruct = @toStruct;
    function s_X = toStruct(X)
        % returns the point components as two fields:
        %  - R is a stiefelstackedfactory, p x d x n
        %  - t is a euclideanfactory, n x p
        % - l is a euclideanfactory, p x m
        % - d is a obliquefactory, p x r
        % this is the inverse function of toStacked
        s_X.R = X(all_R_idxs,:);
        s_X.t = X(all_t_idxs,:);

        if num_landmarks > 0
            s_X.l = X(all_l_idxs,:);
        end
        if num_ranges > 0
            s_X.d = X(all_d_idxs,:)';
        end
    end

    M.toStacked = @toStacked;
    function X = toStacked(s_X)
        % returns the point components stacked in the default fashion
        % this is the inverse function of toStruct
        X = zeros(full_mat_len,lifted_dim);
        X(all_R_idxs,:) = s_X.R;
        X(all_t_idxs,:) = s_X.t;

        % only run this if num_landmarks > 0
        if num_landmarks > 0
            X(all_l_idxs,:) = s_X.l;
        end

        % only run this if num_ranges > 0
        if num_ranges > 0
            X(all_d_idxs,:) = s_X.d';
        end
    end

    % define standard Manopt fields
    cartan_component = sprintf('{St(%d, %d) x R^%d}^%d ', lifted_dim,dim,lifted_dim,num_poses);
    landmark_component = sprintf('{R^%d}^%d', lifted_dim,num_landmarks);
    range_component = sprintf('Oblique(%d, %d)', lifted_dim,num_ranges);

    % full string is the concatenation of the components
    M.name = @() sprintf('%s x %s x %s', cartan_component, landmark_component, range_component);
    M.base_dim = dim;
    M.lifted_dim = lifted_dim;

    M.dim = M_.dim;

    M.inner = @(x, d1, d2) d1(:).'*d2(:);

    M.norm = @(x, d) norm(d(:));

%     M.dist = @(x, y) error('RaSlamManifoldFactory.dist not implemented yet.');

    M.typicaldist = @() sqrt(M.dim());

    M.proj = @(X, U) toStacked( M_.proj(toStruct(X),toStruct(U)) );
    M.tangent = M.proj;

    M.egrad2rgrad = M.proj;
    M.ehess2rhess = @ehess2rhess;
    function rhess = ehess2rhess(X, egrad, ehess, Xdot)
        rhess = toStacked( M_.ehess2rhess( toStruct(X),...
                                        toStruct(egrad),...
                                        toStruct(ehess),...
                                        toStruct(Xdot) ) );
    end

    M.retr = @retraction;
    function Y = retraction(X, U, t)
        if nargin < 3
        t = 1.0;
        end
        Y = toStacked( M_.retr(toStruct(X),toStruct(U),t) );
    end
    M.exp  = @exponential;
    function Y = exponential(X, U, t)
        if nargin < 3
        t = 1.0;
        end
        Y = toStacked( M_.exp( toStruct(X),toStruct(U),t) );
    end

    M.hash = @(X) ['z' hashmd5(X(:))];

    M.rand = @() toStacked( M_.rand() );
    M.randvec = @(X) toStacked( M_.randvec( toStruct(X) ) );

    M.lincomb = @matrixlincomb;

    M.zeros = @() zeros(full_mat_len,lifted_dim);
    M.zerovec = @(x) zeros(full_mat_len, lifted_dim);
    M.transp = @(x1, x2, dim) M.proj(x2, dim);

    M.vec = @(x, u_mat) vec( transpose(u_mat) ); % vectorize transposed version
    M.mat = @(x, u_vec) transpose( reshape(u_vec,[lifted_dim,full_mat_len]) );
    M.vecmatareisometries = @() true;

    % create convenient tools for accessing manifold point data
    % Define list of indeces for different subgroups of variables:
    % all row-indeces in the stacked representation X\in R^{(d+1)n x p}
    all_R_idxs = problem.all_R_idxs;
    m_R_idxs   = reshape(all_R_idxs,dim,num_poses);

    all_t_idxs = problem.all_t_idxs;
    m_t_idxs   = all_t_idxs;

    all_idxs = 1:(num_poses*(dim+1));
    m_T_idxs   = reshape(all_idxs,dim+1,num_poses);



    % accessor functions
    % Note: Stacked versions are column block-vectors (components are trans)
    %       Component versions are straight (not transposed)
    M.get_R  = @(X) X(all_R_idxs,:);
    M.get_Ri = @(X,i) transpose( X(m_R_idxs(:,i),:) );
    M.get_t  = @(X) X(all_t_idxs,:);
    M.get_ti = @(X,i) transpose( X(m_t_idxs(:,i),:) );
    M.get_Ti = @(X,i) transpose( X(m_T_idxs(:,i),:) );

    if num_landmarks > 0
        all_l_idxs = problem.all_l_idxs;
        m_l_idxs   = all_l_idxs;
        M.get_l  = @(X) X(all_l_idxs,:);
        M.get_li = @(X,i) transpose( X(m_l_idxs(:,i),:) );
    end
    if num_ranges > 0
        all_d_idxs = problem.all_d_idxs;
        m_d_idxs = all_d_idxs;
        M.get_d  = @(X) X(all_d_idxs,:);
        M.get_di = @(X,i) transpose( X(m_d_idxs(:,i),:) );
    end


end
