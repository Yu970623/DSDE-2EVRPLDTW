function [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive( ...
    ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
    newObj,newVlt,newCus,newSat,newCap)
%UPDATEARCHIVE 维护跨迭代的非支配可行解档案（两目标 + 约束违反）
%
% 说明：
%   1) 兼容“单个解”与“一批解（整代种群）”两种调用方式；
%   2) 内部仅对违反量 newVlt == 0 的可行解进行非支配更新；
%   3) 函数末尾统一做一次“去重 + 再检查非支配”，保证输出为
%      不重复的非支配解集。
%
% 输入：
%   ArchObj  : [k x 2] 已存档解的目标值（f1,f2）
%   ArchVlt  : [k x 1] 已存档解的总违反量（fvlt）
%   ArchCus  : {k x 1} 已存档解的客户序列 cell
%   ArchSat  : {k x 1} 已存档解的站点序列 cell
%   ArchCap  : {k x 1} 已存档解的卫星载重 cell
%
%   --- 单个解调用（沿用你原来的写法） ---
%   newObj   : [1 x 2] 新解的目标值 (f1,f2)
%   newVlt   : 标量，新解的总违反量 fvlt
%   newCus   : 行向量，新解的客户序列
%   newSat   : 行向量，新解的站点序列
%   newCap   : 行向量，新解的卫星载重
%
%   --- 批量种群调用（每一代的所有解） ---
%   newObj   : [N x 2]，N 为该代中要更新档案的个体数
%   newVlt   : [N x 1] 各个体的总违反量 fvlt
%   newCus   : {N x 1}，newCus{i} 为第 i 个个体的客户序列
%   newSat   : {N x 1}，newSat{i} 为第 i 个个体的卫星序列
%   newCap   : {N x 1}，newCap{i} 为第 i 个个体的卫星载重

    % -------- 0) 空档案初始化 --------
    if nargin == 0
        ArchObj = [];
        ArchVlt = [];
        ArchCus = {};
        ArchSat = {};
        ArchCap = {};
        return;
    end

    % -------- 1) 判定“单个解”还是“一批解” --------
    isBatch = (size(newObj,1) > 1) || (numel(newVlt) > 1) || iscell(newCus);

    if isBatch
        % ===== 批量模式：一整代种群，一次性更新 =====
        [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = ...
            updateArchive_batch(ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                                newObj,newVlt,newCus,newSat,newCap);
    else
        % ===== 单个解模式：保持原有用法 =====
        [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = ...
            updateArchive_single(ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                                 newObj,newVlt,newCus,newSat,newCap);
    end

    % -------- 2) 最后统一做一次“去重 + 再检查非支配” --------
    [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = ...
        archive_cleanup(ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap);
end


%% ================== 单个解更新：与你原来的逻辑基本相同 ==================
function [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive_single( ...
    ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
    newObj,newVlt,newCus,newSat,newCap)

    % 只保留可行解（违反量为 0）
    if newVlt ~= 0
        return;
    end

    if isempty(ArchObj)
        % 第一个可行解直接加入档案
        ArchObj = newObj(:).';
        ArchVlt = newVlt;
        ArchCus = {newCus};
        ArchSat = {newSat};
        ArchCap = {newCap};
        return;
    end

    % ---- 1) 检查完全重复：目标 + 违反量 + 路径完全一致则不加入 ----
    k = size(ArchObj,1);
    for i = 1:k
        if ArchVlt(i) == newVlt && ...
           all(ArchObj(i,:) == newObj(:).')
            if isequal_row(ArchCus{i},newCus) && ...
               isequal_row(ArchSat{i},newSat) && ...
               isequal_row(ArchCap{i},newCap)
                % 完全重复，直接返回
                return;
            end
        end
    end

    % ---- 2) 非支配判定（只在可行解之间比较） ----
    feasMask = ArchVlt == 0;
    ArchObjF = ArchObj(feasMask,:);
    ArchVltF = ArchVlt(feasMask);

    dominatedByNew = false(size(ArchObjF,1),1);
    isDominatedByArchive = false;

    for i = 1:size(ArchObjF,1)
        fOld = ArchObjF(i,:);
        % old 支配 new ?
        if all(fOld <= newObj) && any(fOld < newObj)
            isDominatedByArchive = true;
            break;
        end
        % new 支配 old ?
        if all(newObj <= fOld) && any(newObj < fOld)
            dominatedByNew(i) = true;
        end
    end

    % 新解被支配，则不加入
    if isDominatedByArchive
        return;
    end

    % ---- 3) 删除被新解支配的旧解 ----
    idxF    = find(feasMask);
    idxDrop = idxF(dominatedByNew);

    ArchObj(idxDrop,:) = [];
    ArchVlt(idxDrop)   = [];
    ArchCus(idxDrop)   = [];
    ArchSat(idxDrop)   = [];
    ArchCap(idxDrop)   = [];

    % ---- 4) 加入新解 ----
    ArchObj = [ArchObj; newObj(:).'];
    ArchVlt = [ArchVlt; newVlt];
    ArchCus = [ArchCus; {newCus}];
    ArchSat = [ArchSat; {newSat}];
    ArchCap = [ArchCap; {newCap}];
end


%% ================== 批量更新：整代种群一次性传入 ==================
function [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = updateArchive_batch( ...
    ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
    newObj,newVlt,newCus,newSat,newCap)

    % 统一规格：
    % newObj: [N x 2]
    if size(newObj,2) ~= 2
        error('newObj 在批量模式下必须为 [N x 2]。');
    end
    N = size(newObj,1);

    % newVlt: [N x 1]
    newVlt = newVlt(:);
    if numel(newVlt) ~= N
        error('newVlt 长度必须与 newObj 行数一致。');
    end

    % newCus/newSat/newCap：一律转为 {N x 1} cell，每个 cell 是一条路径
    if ~iscell(newCus)
        newCus = num2cell(newCus,2);
    end
    if ~iscell(newSat)
        newSat = num2cell(newSat,2);
    end
    if ~iscell(newCap)
        newCap = num2cell(newCap,2);
    end
    if numel(newCus) ~= N || numel(newSat) ~= N || numel(newCap) ~= N
        error('newCus/newSat/newCap 的元素个数必须与 newObj 行数一致。');
    end

    % 只对可行解（违反量为 0）进行档案更新
    feasMaskNew = (newVlt == 0);
    idxFeas     = find(feasMaskNew);

    for t = 1:numel(idxFeas)
        i = idxFeas(t);
        [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = ...
            updateArchive_single(ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap, ...
                                 newObj(i,:),newVlt(i), ...
                                 newCus{i},newSat{i},newCap{i});
    end
end


%% ================== 档案最终清理：去重 + 再检查非支配 ==================
function [ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap] = archive_cleanup( ...
    ArchObj,ArchVlt,ArchCus,ArchSat,ArchCap)

    if isempty(ArchObj)
        return;
    end

    % 1) 只保留可行解（违反量 = 0）
    feasMask = (ArchVlt == 0);
    ArchObj  = ArchObj(feasMask,:);
    ArchVlt  = ArchVlt(feasMask);
    ArchCus  = ArchCus(feasMask);
    ArchSat  = ArchSat(feasMask);
    ArchCap  = ArchCap(feasMask);

    if isempty(ArchObj)
        return;
    end

    % 2) 按 [f1,f2,fvlt] 去重（同一目标值视为重复，只保留一条）
    key  = [ArchObj, ArchVlt];
    [~,ia] = unique(key,'rows','stable');
    ArchObj = ArchObj(ia,:);
    ArchVlt = ArchVlt(ia);
    ArchCus = ArchCus(ia);
    ArchSat = ArchSat(ia);
    ArchCap = ArchCap(ia);

    % 3) 再做一遍整体的非支配筛选，确保最终为“全局非支配解集”
    k = size(ArchObj,1);
    isDom = false(k,1);
    for i = 1:k
        if isDom(i), continue; end
        for j = i+1:k
            if isDom(j), continue; end
            fi = ArchObj(i,:);
            fj = ArchObj(j,:);
            if all(fi <= fj) && any(fi < fj)
                isDom(j) = true;
            elseif all(fj <= fi) && any(fj < fi)
                isDom(i) = true;
                break;
            end
        end
    end
    keep  = ~isDom;
    ArchObj = ArchObj(keep,:);
    ArchVlt = ArchVlt(keep);
    ArchCus = ArchCus(keep);
    ArchSat = ArchSat(keep);
    ArchCap = ArchCap(keep);
end


%% ===== 辅助函数：比较两个行向量是否完全相等 =====
function tf = isequal_row(a,b)
    a = a(:).'; b = b(:).';
    tf = (numel(a)==numel(b)) && all(a==b);
end
