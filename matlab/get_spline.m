function [S, dist] =  get_spline( x, y, theta, n_pts )
global PLOT_FLAG

% S: spline polynomial form, accumulated arc length for each interpolation point and coordinates for each point; 
% dist: arc length
    for i=1:(size(x,2)-1)
        [S(i).k,S(i).dk, S(i).L,iter] = buildClothoid( x(i), y(i), theta(i),...
                                            x(i+1), y(i+1), theta(i+1));
    end

    n_pts_clothoid = 20;
    points = [];
    total_length = 0;
    for i=1:(size(x,2)-1)
        add_points = pointsOnClothoid( x(i), y(i), theta(i), S(i).k, S(i).dk, S(i).L, n_pts_clothoid);
        if ( i == 1)
            points = add_points;
        else
            add_points(:,1) = [];
            points = [points, add_points];
        end
        total_length = total_length + S(i).L;
    end

    dist = total_length / n_pts;
    pt = interparc(n_pts,points(1,:),points(2,:),'spline');
    pt = pt';
    
    s = 0:dist:(n_pts-1)*dist;
    S = spline(s, pt)

    if PLOT_FLAG
        plot(x,y, '*k','LineWidth' ,5 );
        hold on
        plot( points(1,:), points(2,:), '-xr' );
        axis equal
        plot( pt(1,:), pt(2,:), '-ob' );

        interpolated = ppval(S, linspace(0,(n_pts-1)*dist,101));
        plot(interpolated(1,:), interpolated(2,:))
        legend('Spline points','Points on computed clothoid','Equally distributed interpolated spline points')
        title('Fitted spline with equally distributed samples')
    end
end