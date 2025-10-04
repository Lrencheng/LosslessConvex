function [x,y]=initialize_trajectory(params)
    N=params.N;
    x=zeros(N,1);
    y=zeros(N,1);
    waypoints=[0 0;
               10 -0.5;
               14 5.5;
               25 6;];
    n=size(waypoints,1);%航点个数
    seg=n-1;
    len=zeros(seg,1);
    sampleNum=zeros(seg,1);%各段采样个数
    for i=1:seg
        vec=[waypoints(i+1,1)-waypoints(i,1),waypoints(i+1,2)-waypoints(i,2)];
        len(i)=norm(vec);
    end
    total=sum(len);
    for i=1:seg-1
        sampleNum(i)=fix(N*len(i)/total);
    end
    sampleNum(seg)=N-sum(sampleNum(1:seg-1));
    
    for j=1:seg-1
        deta_x=(waypoints(j+1,1)-waypoints(j,1))/sampleNum(j);
        deta_y=(waypoints(j+1,2)-waypoints(j,2))/sampleNum(j);
        vec_x=zeros(sampleNum(j),1);
        vec_y=zeros(sampleNum(j),1);
        for k=1:sampleNum(j)
            vec_x(k)=waypoints(j,1)+deta_x*(k-1);
            vec_y(k)=waypoints(j,2)+deta_y*(k-1);
        end
        x=[x;vec_x];
        y=[y;vec_y];
    end
    deta_x=(waypoints(n,1)-waypoints(n-1,1))/(sampleNum(seg)-1);
    deta_y=(waypoints(n,2)-waypoints(n-1,2))/(sampleNum(seg)-1);
    vec_x=zeros(sampleNum(seg),1);
    vec_y=zeros(sampleNum(seg),1);
    for k=1:sampleNum(seg)
        vec_x(k)=waypoints(n-1,1)+deta_x*(k-1);
        vec_y(k)=waypoints(n-1,2)+deta_y*(k-1);
    end
    x=[x;vec_x];
    y=[y;vec_y];
    save('initial_state_profile.mat','x','y');
end