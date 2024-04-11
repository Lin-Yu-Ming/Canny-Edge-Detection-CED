`timescale 1ns/10ps

module  CED(
	input  clk,
	input  rst,
	input  enable,
	output reg ird,
	input  reg [7:0] idata,	
	output reg [13:0] iaddr,
	output reg cwr_mag_0,
	output reg [12:0] cdata_mag_wr0,
	output reg crd_mag_0,
	input  [12:0] cdata_mag_rd0,
	output reg [13:0] caddr_mag_0,
	output reg cwr_ang_0,
	output reg [12:0] cdata_ang_wr0,
	output reg crd_ang_0,
	input  [12:0] cdata_ang_rd0,
	output reg [13:0] caddr_ang_0,
	output reg cwr1,
	output reg [12:0] cdata_wr1,
	output reg crd1,
	input  [12:0] cdata_rd1,
	output reg [13:0] caddr_1,
	output reg cwr2,
	output reg [12:0] cdata_wr2,
	output reg [13:0] caddr_2,
	output reg done
);

reg [5:0] state,next_state,i;
reg [7:0] Grayscale_temp[0:8];
reg signed [12:0] magnitude_L0[0:8];
reg [12:0] angle_L0[0:8];
reg signed [12:0] Gx,Gy,magnitude;
reg signed [12:0] Gx_abs,Gy_abs;
reg [15:0] n,j;
reg [12:0]angle;
reg signed[28:0] Gy_temp,angle_tmp, Gy_temp1;
parameter INIT=5'd0,GRAYSCALE_ADDR=5'd1,WAIT=5'd2,GRAYSCALE_DATA=5'd3,
          LAYER0=5'd4,INIT1=5'd5,LAYER1_ADDR=5'd6,WAIT1=5'd7,LAYER1_DATA=5'd8,
		  LAYER1_DATA1=5'd9,NMS=5'd10,INIT2=5'd11,LAYER2_ADDR=5'd12,
		  WAIT2=5'd13,LAYER2_DATA=5'd14,LAYER2_DATA1=5'd15,HT=5'd16,FINISHED=5'd17;

always @(posedge clk or posedge rst ) begin
  if(rst) state=INIT;
  else state=next_state;
end

always @(*) begin
	case (state)
		INIT:begin
		  next_state=(enable)?GRAYSCALE_ADDR:INIT;
		end 
		
		GRAYSCALE_ADDR:begin
		  next_state=WAIT;
		end

        WAIT:begin
		   next_state=GRAYSCALE_DATA;
		end

        GRAYSCALE_DATA:begin
			next_state=(i>8)?LAYER0:GRAYSCALE_ADDR;  
		end

		LAYER0:begin
		   next_state=(j>15875)?INIT1:GRAYSCALE_ADDR;
		end
 
        INIT1:begin
          next_state=LAYER1_ADDR;
        end

        LAYER1_ADDR:begin
            next_state=WAIT1;
	    end 

        WAIT1:begin
		   if((n>0&&n<125)||(n%126==0)||(n%126==125)||(n>15750&&n<15875))next_state=LAYER1_DATA1;
		   else next_state=LAYER1_DATA;
		end

		LAYER1_DATA:begin
		  next_state=(i>8)?NMS:LAYER1_ADDR; 
		end

		LAYER1_DATA1:begin
          next_state=LAYER1_ADDR;
		end

        NMS:begin
		  next_state=(n>15875)?INIT2:LAYER1_ADDR;
		end

		INIT2:begin
		  next_state=LAYER2_ADDR;
		end

		LAYER2_ADDR:begin
		  next_state=WAIT2;
		end

		WAIT2:begin
		  if((n>0&&n<125)||(n%126==0)||(n%126==125)||(n>15750&&n<15875))next_state=LAYER2_DATA1;
		  else next_state=LAYER2_DATA;
		end

		LAYER2_DATA:begin
		  next_state=(i>8)?HT:LAYER2_ADDR; 
		end

		LAYER2_DATA1:begin
          next_state=LAYER2_ADDR;
		end

		HT:begin
		  next_state=(n>15875)?FINISHED:LAYER2_ADDR;
		end
			
		default:begin
		end 
	endcase	
end

always @(posedge clk) begin
		case (state)
		INIT:begin
		  n=0;
		  i=0;
		  j=0;
		  done=0;
		  angle=0;
		  angle_tmp=0;
		end 
		
		GRAYSCALE_ADDR:begin
		  ird=1;
		  cwr_ang_0=0;
		  cwr_mag_0=0;
		  if (i==0)iaddr=n;
		  else if(i==1)iaddr=n+1;
		  else if(i==2)iaddr=n+2;
		  else if(i==3)iaddr=n+128;
		  else if(i==4)iaddr=n+129;
		  else if(i==5)iaddr=n+130;
		  else if(i==6)iaddr=n+256;
		  else if(i==7)iaddr=n+257;
		  else iaddr=n+258; 
		end
        
		WAIT:begin
		  angle=0;
		  angle_tmp=0;
		end
		
        GRAYSCALE_DATA:begin
		  Grayscale_temp[i]=idata;
		  i=i+1;
		end

		LAYER0:begin
		  i=0;
		  ird=0;
		  cwr_ang_0=1;
		  cwr_mag_0=1;
		  n=(n%128==125)?n+3:n+1;
		  
		  Gx=-(Grayscale_temp[0])+Grayscale_temp[2]-(Grayscale_temp[3]<<1)+
		     (Grayscale_temp[5]<<1)-Grayscale_temp[6]+Grayscale_temp[8];
		  Gy=-Grayscale_temp[0]-(Grayscale_temp[1]<<1)-Grayscale_temp[2]+
		     Grayscale_temp[6]+(Grayscale_temp[7]<<1)+Grayscale_temp[8];

          Gx_abs=(Gx[12]==1)?-Gx:Gx;
          Gy_abs=(Gy[12]==1)?-Gy:Gy;
		  magnitude=Gx_abs+Gy_abs;

          angle_tmp=(Gx!=0)?(Gy<<16)/Gx:Gx;
		  if(angle_tmp<$signed(29'h00006A09) && angle_tmp>$signed(29'h1FFF95F7))begin                        //0
			if (Gx!=0&&Gy==0)angle=13'h00;
			else if (Gx==0&&Gy!=0)angle=13'h5a;
			else angle=13'h00;
		  end         
		  else if(angle_tmp>$signed(29'h00006A09) && angle_tmp<$signed(29'h00026A09))angle=13'h2d;           //45
		  else if(angle_tmp>$signed(29'h00026A09) || angle_tmp<$signed(29'h1FFD95F7))angle=13'h5a;           //90
		  else angle=13'h87;
		  caddr_mag_0=j;
		  cdata_mag_wr0=magnitude;
		  caddr_ang_0=j;
          cdata_ang_wr0=angle;
		  j=j+1;
		end

        INIT1:begin
		  j=0;
		  n=0;
		  i=0;
		  cwr_ang_0=0;
		  cwr_mag_0=0;
        end

		LAYER1_ADDR:begin
		  if ((n>0&&n<125)||(n%126==0)||(n%126==125)||(n>15750&&n<15875)) begin
		    cwr1=0;
			caddr_1=n;
		  end
          else begin
		    crd_ang_0=1;
		    crd_mag_0=1;
			cwr1=0;
			if (i==0)begin
			  caddr_ang_0=j;
			  caddr_mag_0=j;
			end
		    else if(i==1)begin
			  caddr_ang_0=j+1;
			  caddr_mag_0=j+1;
			end
		    else if(i==2)begin
			  caddr_ang_0=j+2;
			  caddr_mag_0=j+2;
			end
		    else if(i==3)begin
			  caddr_ang_0=j+126;
			  caddr_mag_0=j+126;
			end
		    else if(i==4)begin
			  caddr_ang_0=j+127;
			  caddr_mag_0=j+127;
			end
		    else if(i==5)begin
			  caddr_ang_0=j+128;
			  caddr_mag_0=j+128;
			end
		    else if(i==6)begin
			  caddr_ang_0=j+252;
			  caddr_mag_0=j+252;
			end
		    else if(i==7)begin
			  caddr_ang_0=j+253;
			  caddr_mag_0=j+253;
			end
		    else begin
			  caddr_ang_0=j+254;
			  caddr_mag_0=j+254;
			end
		  end
		end

		LAYER1_DATA:begin
		  angle_L0[i]=cdata_ang_rd0;
		  magnitude_L0[i]=cdata_mag_rd0;
		  i=i+1;
		end

		LAYER1_DATA1:begin
          	cwr1=1;
			cdata_wr1=0;
			n=n+1;
		end

		NMS:begin
		  crd_ang_0=0;
		  crd_mag_0=0;
		  cwr1=1;
		  caddr_1=n;
		  if (angle_L0[4]==13'h00) begin
            cdata_wr1=(magnitude_L0[4]>=magnitude_L0[3]
			          &&magnitude_L0[4]>=magnitude_L0[5])?magnitude_L0[4]:0;			
		  end
		  else if (angle_L0[4]==13'h2d) begin
			cdata_wr1=(magnitude_L0[4]>=magnitude_L0[2]
			          &&magnitude_L0[4]>=magnitude_L0[6])?magnitude_L0[4]:0;	
		  end
          else if (angle_L0[4]==13'h5a) begin
			cdata_wr1=(magnitude_L0[4]>=magnitude_L0[1]
			          &&magnitude_L0[4]>=magnitude_L0[7])?magnitude_L0[4]:0;	
		  end
		  else begin
			cdata_wr1=(magnitude_L0[4]>=magnitude_L0[0]
			          &&magnitude_L0[4]>=magnitude_L0[8])?magnitude_L0[4]:0;
		  end
		  j=(j%126==123)?j+3:j+1;
		  n=n+1;
		  i=0;
		end

		INIT2:begin
		  j=0;
		  n=0;
		  i=0;
          cwr1=0;
		end

		LAYER2_ADDR:begin
		  if ((n>0&&n<125)||(n%126==0)||(n%126==125)||(n>15750&&n<15875)) begin
		    cwr2=0;
			caddr_2=n;
		  end
          else begin
            crd1=1;
			cwr2=0;
			if (i==0)caddr_1=j;
		    else if(i==1)caddr_1=j+1;
		    else if(i==2)caddr_1=j+2;
		    else if(i==3)caddr_1=j+126;
		    else if(i==4)caddr_1=j+127;
		    else if(i==5)caddr_1=j+128;
		    else if(i==6)caddr_1=j+252;
		    else if(i==7)caddr_1=j+253;
		    else caddr_1=j+254;
		  end
		end

		LAYER2_DATA:begin
		  magnitude_L0[i]=cdata_rd1;
		  i=i+1;
		end	

		LAYER2_DATA1:begin
          	cwr2=1;
			cdata_wr2=0;
			n=n+1;
		end
		
		HT:begin
          crd1=0;
		  cwr2=1;
		  caddr_2=n;
          if (magnitude_L0[4]>=13'd100)cdata_wr2=13'd255;
		  else if(magnitude_L0[4]<13'd50)cdata_wr2=13'd0;
		  else begin
			if ((magnitude_L0[0]>=13'd100)||(magnitude_L0[1]>=13'd100)||
			    (magnitude_L0[2]>=13'd100)||(magnitude_L0[3]>=13'd100)||
				(magnitude_L0[5]>=13'd100)||(magnitude_L0[6]>=13'd100)||
				(magnitude_L0[7]>=13'd100)||(magnitude_L0[8]>=13'd100)) begin
				cdata_wr2=13'd255;
			end
			else cdata_wr2=13'd0;
		  end
		  j=(j%126==123)?j+3:j+1;
		  n=n+1;
		  i=0;
		end

		FINISHED:begin
		  done=1;
		end

		default:begin
		end 
	endcase
end
endmodule
