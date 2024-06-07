// g++ `pkg-config --cflags opencv4` main.cpp -o main `pkg-config --libs opencv4`

#include <filesystem>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>
#include <string>

#include "mjpeg_streamer.hpp"
using namespace cv;
using namespace std;
using MJPEGStreamer = nadjieb::MJPEGStreamer;
namespace fs = std::filesystem;

const double scale = 2.0;
int SQUARE_SIZE = 14 * scale;
int write_flg = 1;
double vel = SQUARE_SIZE * 0.25;
int class_div = 3;
const float history_cnt = 10;
int pong_num = 11;

// ポンボールを表す構造体
struct _pong {
    double x;                   // x 座標
    double y;                   // y 座標
    double dx;                  // x 方向の速度
    double dy;                  // y 方向の速度
    double vel;                 // 速度（大きさ）
    int class_num;              // クラス番号
    cv::Scalar color;           // 色
    vector<double> old_x;       // 過去の x 座標を記録するベクトル
    vector<double> old_y;       // 過去の y 座標を記録するベクトル
    int old_size = 1;           // 過去の座標を保持するベクトルのサイズ
    int conflict = 0;           // 衝突フラグ（0: 衝突していない、1: 衝突している）
    double r = SQUARE_SIZE / 2; // 半径
};

// 正方形を表す構造体
struct _square {
    int class_num;     // クラス番号
    int old_class_num; // 前のフレームでのクラス番号
    int life_cnt = 99; // ライフカウント（衝突後のカウント）
};

double sign(double A);                               // 指定された値の符号を返す関数
double rand_std(double ave = 0, double std = 1);     // 正規分布
double rand_uni(double min = 0.0, double max = 1.0); // 一様分布乱数
int checkBoundaryCollision(int image_width, int img_height, _pong &pong);

int updateScoreElement(std::vector<vector<int>> &squares, cv::Mat &image) {
    int dayScore = 0;
    int nightScore = 0;
    for(unsigned int i = 0; i < squares.size(); i++) {
        for(unsigned int j = 0; j < squares[i].size(); j++) {
            if(squares[i][j] == 0)
                dayScore++;
            else if(squares[i][j] == 1)
                nightScore++;
        }
    }

    string str = "dayScore" + to_string(dayScore) + " nightScore" + to_string(nightScore);

    cv::putText(image, str, cv::Point(25, image.rows - 25), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
}

int updateSquareAndBounce(std::vector<vector<_square>> &squares, _pong &pong) {
    // ポンボールの速度を一時的に保持
    double updatedDx = pong.dx;
    double updatedDy = pong.dy;

    // 正方形の一辺の長さに基づいて角度の変化を決定
    double d_angle = (SQUARE_SIZE * 4) / SQUARE_SIZE + 4;

    // ボールの周囲の複数のポイントをチェック
    for(double angle = 0; angle < M_PI * 2; angle += 2 * M_PI / (d_angle)) {
        // チェックする点の座標を計算
        int checkX = pong.x + cos(angle) * pong.r;
        int checkY = pong.y + sin(angle) * pong.r;

        // 正方形のインデックスを計算
        int i = floor(checkX / SQUARE_SIZE);
        int j = floor(checkY / SQUARE_SIZE);

        // 正方形が存在し、ポンボールと同じクラスでない場合
        if(i >= 0 && i < squares.size() && j >= 0 && j < squares[0].size()) {
            if((squares[i][j].class_num != pong.class_num)) {
                // 正方形のクラスを更新
                squares[i][j].class_num = pong.class_num;
                // 反射の方向を角度に基づいて決定
                if(abs(cos(angle)) > abs(sin(angle))) {
                    updatedDx = -updatedDx;
                    pong.conflict = 1;

                } else {
                    updatedDy = -updatedDy;
                    pong.conflict = 1;
                }
            }
        }

        // ポンボールの半径が正方形のサイズの半分でない場合、角度の変化を再計算
        if(pong.r / 2 != SQUARE_SIZE)
            d_angle = (pong.r * 2 * 4) / SQUARE_SIZE + 4;
    }

    // ランダムな角度を追加してポンボールの速度を更新
    double theta = M_PI / 4 * (1 + 0.1 * rand_std());
    pong.dx = sign(updatedDx) * pong.vel * cos(theta);
    pong.dy = sign(updatedDy) * pong.vel * sin(theta);

    return 0;
}

int updateBounceAndBounce(_pong &pong, vector<_pong> &pong_other, int num) {
    // 他のポンボールに対してループを実行
    for(unsigned int n = 0; n < pong_other.size(); n++) {
        // 他のポンボールの座標を取得
        int otherX = pong_other[n].x;
        int otherY = pong_other[n].y;

        int baseX = pong.x;
        int baseY = pong.y;

        // ポンボール同士の距離と角度を計算
        double angle = std::atan2(baseY - otherY, baseX - otherX);
        //        double angle = std::atan2(otherY - baseY, otherX - baseX);
        double dis = sqrt((baseX - otherX) * (baseX - otherX) + (baseY - otherY) * (baseY - otherY));

        // 衝突条件をチェックし、Boll衝突している場合
        if((dis <= (pong_other[n].r + pong.r)) && (n != num)) {

            pong.x = pong_other[n].x + (pong.r + pong_other[n].r) * cos(angle);
            pong.y = pong_other[n].y + (pong.r + pong_other[n].r) * sin(angle);

            // pong_other[n].x = baseX + (pong.r + pong_other[n].r) * cos(angle);
            // pong_other[n].y = baseY + (pong.r + pong_other[n].r) * sin(angle);

            if((pong_other[n].conflict == 0) && (pong.conflict == 0)) {
                // 衝突方向を決定し、反射させる
                if(abs(cos(angle)) > abs(sin(angle))) {
                    pong.dx = -pong.dx;
                    pong_other[n].dx = -pong_other[n].dx;
                    pong.conflict = 1;
                    pong_other[n].conflict = 1;
                } else {
                    pong.dy = -pong.dy;
                    pong_other[n].dy = -pong_other[n].dy;
                    pong.conflict = 1;
                    pong_other[n].conflict = 1;
                }
            }

            else if(pong.conflict == 0) {
                if(abs(cos(angle)) > abs(sin(angle))) {
                    pong.dx = -pong.dx;
                    pong.conflict = 1;

                } else {
                    pong.dy = -pong.dy;
                    pong.conflict = 1;
                }
            } else if(pong_other[n].conflict == 0) {
                if(abs(cos(angle)) > abs(sin(angle))) {
                    pong_other[n].dx = -pong_other[n].dx;
                    pong_other[n].conflict = 1;
                } else {
                    pong_other[n].dy = -pong_other[n].dy;
                    pong_other[n].conflict = 1;
                }
            }
        }
    }

    return 0;
}

int checkBoundaryCollision(int image_width, int img_height, _pong &pong) {
    // 左端に衝突した場合
    if(pong.x + pong.dx < pong.r) {
        // 右方向に反射
        pong.dx = abs(pong.dx);
    }
    // 右端に衝突した場合
    if(pong.x + pong.dx > image_width - pong.r) {
        // 左方向に反射
        pong.dx = -1 * abs(pong.dx);
    }
    // 上端に衝突した場合
    if(pong.y + pong.dy < pong.r) {
        // 下方向に反射
        pong.dy = abs(pong.dy);
    }
    // 下端に衝突した場合
    if(pong.y + pong.dy > img_height - pong.r) {
        // 上方向に反射
        pong.dy = -1 * abs(pong.dy);
    }
    return 0;
}

int main() {

    MJPEGStreamer streamer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    streamer.start(7777);

    VideoWriter writer("Video.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 60.0, Size(320 * scale, 240 * scale), true); // 動画ファイル生成
    cv::Mat image = cv::Mat::zeros(240 * scale, 320 * scale, CV_8UC3);
    int img_width = image.cols;
    int img_height = image.rows;

    vector<cv::Scalar> squares_color;
    squares_color.resize(class_div);

    if(class_div <= 25)
        squares_color.resize(25);

    for(unsigned int i = 0; i < squares_color.size(); i++) {
        squares_color[i] = cv::Scalar(rand_std() * 255, rand_std() * 255, rand_std() * 255);
    }
    squares_color[0] = cv::Scalar(230, 230, 230);
    squares_color[1] = cv::Scalar(100, 100, 100);
    squares_color[2] = cv::Scalar(255, 170, 255);
    squares_color[3] = cv::Scalar(230, 253, 255);
    squares_color[4] = cv::Scalar(255, 128, 248);
    squares_color[5] = cv::Scalar(191, 144, 95);
    squares_color[6] = cv::Scalar(192, 209, 210);
    squares_color[7] = cv::Scalar(191, 227, 202);
    squares_color[8] = cv::Scalar(190, 214, 248);
    squares_color[9] = cv::Scalar(182, 136, 189);
    squares_color[10] = cv::Scalar(189, 174, 0);
    squares_color[11] = cv::Scalar(47, 255, 173);
    squares_color[12] = cv::Scalar(214, 223, 237);
    squares_color[13] = cv::Scalar(178, 165, 236);
    squares_color[14] = cv::Scalar(103, 193, 173);
    squares_color[15] = cv::Scalar(222, 196, 176);
    squares_color[16] = cv::Scalar(217, 93, 151);
    squares_color[17] = cv::Scalar(47, 255, 173);
    squares_color[18] = cv::Scalar(214, 223, 237);
    squares_color[19] = cv::Scalar(49, 124, 144);
    squares_color[20] = cv::Scalar(209, 181, 146);
    squares_color[22] = cv::Scalar(222, 196, 176);
    squares_color[23] = cv::Scalar(232, 134, 143);
    squares_color[24] = cv::Scalar(232, 130, 142);

    // 画像の幅と高さから正方形の数を計算
    int numSquaresX = img_width / SQUARE_SIZE + 1;
    int numSquaresY = img_height / SQUARE_SIZE + 1;

    // 正方形の数を出力
    cout << "numSquaresX" << numSquaresX << endl;
    cout << "numSquaresY" << numSquaresY << endl;

    // 正方形を格納する二次元配列を作成
    std::vector<vector<_square>> squares;
    // X方向の正方形の数だけ行を作成
    squares.resize(numSquaresX);
    // 各行にY方向の正方形の数だけ列を作成
    for(unsigned int i = 0; i < numSquaresX; i++) {
        squares[i].resize(numSquaresY);
    }

    for(unsigned int i = 0; i < numSquaresX; i++) {
        for(unsigned int j = 0; j < numSquaresY; j++) {
            if(i < numSquaresX / 2)
                squares[i][j].class_num = 0;
            else
                squares[i][j].class_num = 1;
        }
    }

    vector<cv::Scalar> class_color;
    class_color.resize(class_div);

    if(class_div <= 25)
        class_color.resize(25);

    for(unsigned int i = 0; i < class_color.size(); i++) {
        class_color[i] = cv::Scalar(rand_std() * 255, rand_std() * 255, rand_std() * 255);
    }

    class_color[0] = cv::Scalar(0, 0, 0);
    class_color[1] = cv::Scalar(255, 255, 255);
    class_color[2] = cv::Scalar(0, 0, 255);

    class_color[3] = cv::Scalar(0, 255, 128);
    class_color[4] = cv::Scalar(128, 255, 0);
    class_color[5] = cv::Scalar(255, 128, 128);
    class_color[6] = cv::Scalar(0, 128, 255);
    class_color[7] = cv::Scalar(0, 128, 255);
    class_color[8] = cv::Scalar(0, 128, 255);

    vector<_pong> pong;
    pong.resize(25);
    int x_div = 1;
    int grid = numSquaresX / x_div;
    for(unsigned int i = 0; i < pong.size(); i++) {

        pong[i].x = img_width * rand_std();
        pong[i].y = img_height * rand_std();

        double _rand = rand_std();
        pong[i].dx = vel * _rand / abs(_rand);
        _rand = rand_std();
        pong[i].dy = vel * _rand / abs(_rand);
        pong[i].class_num = i;
        pong[i].color = class_color[i];
        pong[i].vel = sqrt(pong[i].dx * pong[i].dx + pong[i].dy * pong[i].dy);
        pong[i].r = SQUARE_SIZE / 2.0;
    }

    int flg = true;
    if(flg) {
        pong[0].x = 160;
        pong[0].y = 240;
        pong[0].class_num = 0;
        pong[0].r = SQUARE_SIZE / 2 * sqrt(9 * 9);
        pong[0].color = class_color[0];

        pong[1].x = 320 + 100;
        pong[1].y = 120;
        pong[1].class_num = 1;
        pong[1].vel = vel;
        pong[1].color = class_color[1];

        pong[2].x = 320 + 150;
        pong[2].y = 150;
        pong[2].class_num = 1;
        pong[2].vel = vel;
        pong[2].color = class_color[1];

        pong[3].x = 320 + 200;
        pong[3].y = 180;
        pong[3].class_num = 1;
        pong[3].vel = vel;
        pong[3].color = class_color[1];

        pong[4].x = 320 + 125;
        pong[4].y = 140;
        pong[4].class_num = 1;
        pong[4].vel = vel;
        pong[4].color = class_color[1];

        pong[5].x = 320 + 250;
        pong[5].y = 130;
        pong[5].class_num = 1;
        pong[5].color = class_color[1];

        pong[6].x = 320 + 70;
        pong[6].y = 120;
        pong[6].class_num = 1;
        pong[6].color = class_color[1];

        pong[7].x = 320 + 220;
        pong[7].y = 100;
        pong[7].class_num = 1;
        pong[7].color = class_color[1];

        pong[8].x = 320 + 90;
        pong[8].y = 140;
        pong[8].class_num = 1;
        pong[8].color = class_color[1];

        pong[9].x = 320 + 110;
        pong[9].y = 50;
        pong[9].class_num = 1;
        pong[9].color = class_color[1];

        pong[10].x = 320 + 110;
        pong[10].y = 150 + 200;
        pong[10].class_num = 2;
        pong[10].color = class_color[2];
        pong[10].vel = vel * sqrt(9);
    }

    pong.resize(pong_num);

    // 初期位置の干渉回避
    for(unsigned int i = 0; i < pong.size(); i++) {
        for(unsigned int j = 0; j < pong.size(); j++) {
            // ポンボール i の座標を取得
            int cx = pong[i].x;
            int cy = pong[i].y;

            // ポンボール j の座標を取得
            int dx = pong[j].x;
            int dy = pong[j].y;

            // ポンボール間の距離を計算
            double dis = sqrt((cx - dx) * (cx - dx) + (cy - dy) * (cy - dy));
            double angle = std::atan2(dy - cy, dx - cx);

            // ポンボール同士が干渉している場合
            if((dis <= (pong[i].r + pong[j].r)) && (i != j)) {
                // ポンボールの位置を変更して干渉を解消する
                pong[i].x = pong[i].x + (pong[i].r + pong[j].r) * cos(angle);
                pong[i].y = pong[i].y + (pong[i].r + pong[j].r) * sin(angle);
            }

            // 画面の境界に配置されるようにポンボールの位置を調整
            if(pong[i].x < pong[i].r) {
                pong[i].x = pong[i].r;
            }
            if(pong[i].x > img_width - pong[i].r) {
                pong[i].x = img_width - pong[i].r;
            }
            if(pong[i].y < pong[i].r) {
                pong[i].y = pong[i].r;
            }
            if(pong[i].y > img_height - pong[i].r) {
                pong[i].y = img_height - pong[i].r;
            }
        }
    }

    cout << "Started ... " << endl;

    while(1) {

        // 正方形の古いクラス番号を更新する
        for(unsigned int i = 0; i < squares.size(); i++) {
            for(unsigned int j = 0; j < squares[i].size(); j++) {
                squares[i][j].old_class_num = squares[i][j].class_num;
            }
        }

        // 各ポンボールの衝突フラグをリセットする
        for(unsigned int i = 0; i < pong.size(); i++) {
            pong[i].conflict = 0;
        }

        // ポンボールの位置と状態を更新する
        for(unsigned int i = 0; i < pong.size(); i++) {
            // ポンボールの位置を更新する
            pong[i].x += pong[i].dx;
            pong[i].y += pong[i].dy;
            // 正方形との衝突を処理する
            updateSquareAndBounce(squares, pong[i]);
            // ポンボール同士の衝突を処理する
            updateBounceAndBounce(pong[i], pong, i);
            // 画像の境界での反射をチェックする
            checkBoundaryCollision(image.cols, image.rows, pong[i]);
        }

        // ライフカウントを更新する
        for(unsigned int i = 0; i < squares.size(); i++) {
            for(unsigned int j = 0; j < squares[i].size(); j++) {
                // 正方形が前回のクラスと異なる場合、ライフカウントをリセット
                if(squares[i][j].class_num != squares[i][j].old_class_num) {
                    squares[i][j].life_cnt = 0;
                } else {
                    // 正方形が前回のクラスと同じ場合、ライフカウントを増やす
                    squares[i][j].life_cnt++;
                }
            }
        }

        // 正方形を表示する
        for(unsigned int i = 0; i < squares.size(); i++) {
            for(unsigned int j = 0; j < squares[i].size(); j++) {
                // 正方形の色を初期化
                cv::Scalar color = cv::Scalar(0, 0, 0);
                // 正方形のクラスに応じて色を設定
                for(unsigned int n = 0; n < class_div; n++) {
                    if(squares[i][j].class_num == n) {
                        color = squares_color[n];
                    }
                }
                // ライフカウントに応じて色を変化させる
                if(squares[i][j].life_cnt < history_cnt) {
                    double ratio = (squares[i][j].life_cnt) / history_cnt;
                    color = cv::Scalar(0, 0, 255) * (1.0 - ratio) + color * (ratio);
                }

                // 正方形を描画
                cv::rectangle(image, cv::Point(i * SQUARE_SIZE, j * SQUARE_SIZE), cv::Point(i * SQUARE_SIZE + SQUARE_SIZE, j * SQUARE_SIZE + SQUARE_SIZE), color, cv::FILLED);
            }
        }

        // ポンボールを表示する
        for(unsigned int i = 0; i < pong.size(); i++) {
            // 各ポンボールの位置に円を描画
            // 半径はポンボールオブジェクトの半径(pong[i].r)になります
            cv::circle(image, cv::Point(pong[i].x, pong[i].y), pong[i].r, pong[i].color, -1);
        }

        // ポンボールの過去の軌跡を描画する
        for(unsigned int i = 0; i < pong.size(); i++) {
            // ポンボールの過去のx座標とy座標を記録する
            pong[i].old_x.push_back(pong[i].x);
            pong[i].old_y.push_back(pong[i].y);

            // 記録された座標数が規定のサイズを超えている場合、古い座標を削除する
            if(pong[i].old_x.size() > pong[i].old_size)
                pong[i].old_x.erase(pong[i].old_x.begin());

            if(pong[i].old_y.size() > pong[i].old_size)
                pong[i].old_y.erase(pong[i].old_y.begin());

            // 過去の座標を結んで軌跡を描画する
            for(unsigned int n = 0; n < pong[i].old_x.size() - 1; n++) {
                // 軌跡の色を計算する
                double ratio = (double)n / (pong[i].old_x.size() - 1);
                cv::Scalar color = pong[i].color * (ratio) + squares_color[i] * (1 - ratio);
                // 軌跡を描画する
                cv::line(image, cv::Point(pong[i].old_x[n], pong[i].old_y[n]), cv::Point(pong[i].old_x[n + 1], pong[i].old_y[n + 1]), color, 2);
                // 過去の軌跡に円を描画する場合は次の行のコメントを解除
                // cv::circle(image, cv::Point(pong[i].old_x[n], pong[i].old_y[n]), SQUARE_SIZE / 2*(ratio), pong[i].color, -1);
            }
        }

        // 画像を表示する
        imshow("Display window", image);

        // JPEG形式で画像をエンコードする
        std::vector<uchar> buff_bgr;
        cv::imencode(".jpg", image, buff_bgr, params);

        // エンコードされたバイト列を文字列に変換して配信する
        streamer.publish("/bgr", std::string(buff_bgr.begin(), buff_bgr.end()));

        // キーボード入力を待ちます（引数の数値は待ち時間をミリ秒単位で指定します）
        char key = (char)waitKey(1000 / 100);

        // キー入力に応じて処理を実行します
        if(write_flg)
            writer << image; // 動画を書き込むためのライターに画像を追加します（write_flg が true の場合のみ）
        if(key == 27)        // ESC キーが押された場合はループを抜けます
            break;

        if(key == 'a' || key == 'A') { // 'a' または 'A' キーが押された場合の処理

            // 最初の 10 個のポンボールの速度を倍にします
            for(unsigned int i = 0; i < 10; i++) {
                pong[i].dx *= 2.0;
                pong[i].dy *= 2.0;
                pong[i].vel *= 2.0;
            }

            // 次の 10 個のポンボールの速度を半分にします
            for(int i = 10; i < 20; i++) {
                pong[i].dx *= 0.5;
                pong[i].dy *= 0.5;
                pong[i].vel *= 0.5;
            }
        }

        if(key == 's' || key == 'S') {
        }
    }
    streamer.stop();
    cout << "Write complete !" << endl;
    return 0;
}

// 指定された平均値と標準偏差を持つ正規分布に従う乱数を生成する関数
double rand_std(double ave, double std) {
    // 乱数生成器のシードを生成
    static std::random_device seed_gen;
    // メルセンヌ・ツイスター擬似乱数生成器を初期化
    static std::mt19937 engine(seed_gen());

    // 指定された平均値と標準偏差を持つ正規分布に従う乱数を生成する分布オブジェクトを定義
    std::normal_distribution<float> dist2(ave, std);
    // 正規分布に従う乱数を生成し、変数 rand に格納
    double rand = dist2(engine);
    // 生成された乱数を返す
    return rand;
}

// 指定された最小値と最大値の範囲で一様分布する実数を生成する関数
double rand_uni(double min, double max) {
    // 乱数生成器のシードを生成
    static std::random_device seed_gen;
    // メルセンヌ・ツイスター擬似乱数生成器を初期化
    static std::mt19937 engine(seed_gen());

    // 指定された範囲で一様分布する実数を生成する分布オブジェクトを定義
    std::uniform_real_distribution<float> dist1(min, max);
    // 実数乱数を生成し、変数 rand に格納
    double rand = dist1(engine);
    // 生成された乱数を返す
    return rand;
}

// 指定された値の符号を返す関数
double sign(double A) {
    // もし A が 0 ならば
    if(A == 0)
        // 0 を返す
        return 0;
    // そうでなければ（つまり A が 0 でない場合）
    else
        // A をその絶対値で割った結果を返すことで、A の符号を取得する
        return A / abs(A);
}
